/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
***************************************************************************/
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 27 };
/********************************* Logger *************************/
#include "bt_fw_types.h"
#include "bz_fw_isoch.h"
#include "lmp_defines.h"
#include "bb_driver.h"
#include "bt_fw_hci_external_defines.h"
#include "platform.h"
#include "logger.h"
#include "lc_internal.h"
#include "UartPrintf.h"
#include "timer.h"
#include "bt_fw_os.h"
#include "mem.h"
#include "bzdma.h"
#include "dma_usb.h"
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
UINT16 g_esco_print_log =0;
#endif

#ifdef SCO_OVER_HCI
/********************** Global variable definations ***********************/
SCO_ISOCH_MAP bz_isoch_sco_map[ISOCH_SCO_MAX_CONNS]; /* mapping table between
                                                        usb iso ep and sco 
                                                        channel */
volatile UCHAR bz_sco_pkt_size = 48;    /* sco packet size */
UCHAR bz_isoch_conns = 0;                           /* current amount of of iso 
                                                       channels */
BOOLEAN	bz_isoch_is_usb_ready = FALSE;  /* usb iso channel is ready ? */

UINT16 bz_sync_ch_conn_handle = 0;      /* connection handle */
#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_bz_isoch_send_data_to_host = NULL;
#endif
#endif
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
ISOCH_SCO_HCI_RX_QUEUE_MANAGER bz_isoch_queue_manager;
#endif

SCO_TX_ENTRY_TYPE *sco_tx_entry_free = NULL;
SCO_TX_ENTRY_TYPE sco_tx_entry_unit[BT_FW_TOTAL_SYNCHRONOUS_TX_ENTRY];

SECTION_SRAM UINT8 sco_tx_zero_buf[64];
			
#ifdef _SCO_SEND_SINE_TONE_PKT_
extern const UINT16 Bzdma_1KHz_Sine_16BIT_SAMPLE[8];
#endif

extern POOL_ID	synchronous_data_to_host_pool_id;
extern POOL_ID synchronous_data_to_remote_id;
extern UINT8 g_usb_iso_alternate_change[ISOCH_SCO_MAX_CONNS];
UINT8 bqb_receive_sco_esco_pkt_in_ed_test = FALSE;

/*********************** Function Declarations ****************************/
/**
 * Initializes the iso-q mappings.
 * \param  None.
 * \return None.
 */
void bz_isoch_init_q(void)
{
    UCHAR i;

    for(i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        bz_isoch_sco_map[i].conn_handle = INVALID_CONN_HANDLE;

        /* Host to Controller Data Path */
        bz_isoch_sco_map[i].isoch_tx_q.sco_pkt_wr_ptr = 0;
        bz_isoch_sco_map[i].isoch_tx_q.sco_pkt_rd_ptr = 0;

        /* the sco_pkt_buff is 128 byte ring buffer */
        bz_isoch_sco_map[i].isoch_tx_q.sco_pkt_buff = NULL;
        bz_isoch_sco_map[i].isoch_tx_q.scoch_map = (void*)&bz_isoch_sco_map[i]; 

        /* Controller to Host Data Path */
        bz_isoch_sco_map[i].isoch_rx_q.sco_rx_pkt = NULL;
        bz_isoch_sco_map[i].isoch_rx_q.fragment_length = 0;
        bz_isoch_sco_map[i].isoch_rx_q.fragment_status = ERR_CORRECT_DATA;
        bz_isoch_sco_map[i].isoch_rx_q.scoch_map = (void*)&bz_isoch_sco_map[i]; 

        /* add by austin */
        bz_isoch_sco_map[i].active = 0;
        bz_isoch_sco_map[i].sco_ch_id = 0; 
        bz_isoch_sco_map[i].host_data_8bit = 0;
        bz_isoch_sco_map[i].air_data_8bit = 0;
        bz_isoch_sco_map[i].no_transparent = 0;
        bz_isoch_sco_map[i].cur_id = i;
        bz_isoch_sco_map[i].next_instant = 0xFFFFFFFF;

        bz_isoch_sco_map[i].sco_tx_entry_manager.header = NULL;
        bz_isoch_sco_map[i].sco_tx_entry_manager.ender = NULL;    
        bz_isoch_sco_map[i].sco_tx_entry_manager.pkt_num = 0;
        bz_isoch_sco_map[i].sco_tx_entry_manager.wait_head = NULL;
        bz_isoch_sco_map[i].sco_tx_entry_manager.wait_end = NULL;        
        bz_isoch_sco_map[i].sco_tx_entry_manager.offset = 0;

#ifdef _SCO_SEND_SINE_TONE_PKT_   
        bz_isoch_sco_map[i].sco_tx_entry_manager.sine_tx_index = 0;
        bz_isoch_sco_map[i].sco_tx_entry_manager.tone_src = Bzdma_Sw_SCO_Tx_Buf[i];
        bz_isoch_sco_map[i].sco_tx_entry_manager.tone_repeat_mask = 0x0F >> i;
#endif
    }
    bz_isoch_conns = 0;

    /* init free sco tx entry */
    pf_hci_init_free_sco_tx_entry();   

    /* init sco tx zero buffer in sram */
    memset(sco_tx_zero_buf, 0, 64);

#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
    memset(&bz_isoch_queue_manager, 0, sizeof(ISOCH_SCO_HCI_RX_QUEUE_MANAGER));
#endif
}


/**
 * Allocates iso-queues to a SCO connection, and updates the isochronous
 * pkt length acording to number of connections.
 *
 * \param sync_conn_handle Connection handle of new connection.
 *
 * \return API_SUCCESS if allocation successful, else API_FAILURE.
 */
SCO_ISOCH_MAP* bz_isoch_add_sco_queues(UINT16 sync_conn_handle, UINT8 ce_index)
{
    UCHAR i;
    LMP_CONNECTION_ENTITY *lmp_ce;

    if ((sync_conn_handle == INVALID_CONN_HANDLE) || 
        (ce_index >= LMP_MAX_CE_DATABASE_ENTRIES) ||
        (bz_isoch_conns >= ISOCH_SCO_MAX_CONNS))
    {
        return NULL;
    }

    lmp_ce = &lmp_connection_entity[ce_index];    

    if (bz_isoch_conns == 0)
    {
        UINT16 temp;
        
        /* Flush whole sco tx fifo */
        /* Flush the whole FIFO for removing residual data */
        BB_flush_baseband_SYNC_TX_FIFO(SYNC_FIFO1, 0);
        BB_flush_baseband_SYNC_RX_FIFO(SYNC_FIFO1, 0);

        /* reset internal codec */
        temp = BB_read_baseband_register(VOICE_SETTING_REGISTER);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, temp | BIT9);
        BB_write_baseband_register(VOICE_SETTING_REGISTER, temp);          
    }

    for(i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        if(bz_isoch_sco_map[i].conn_handle == INVALID_CONN_HANDLE)
        {
            DEF_CRITICAL_SECTION_STORAGE;
            
            MINT_OS_ENTER_CRITICAL();

            bz_isoch_sco_map[i].conn_handle = sync_conn_handle;
            bz_isoch_sco_map[i].isoch_tx_q.sco_pkt_wr_ptr = 0;
            bz_isoch_sco_map[i].isoch_tx_q.sco_pkt_rd_ptr = 0;

            bz_isoch_sco_map[i].isoch_rx_q.fragment_length = 0;
            bz_isoch_sco_map[i].isoch_rx_q.fragment_status = ERR_CORRECT_DATA;
            bz_isoch_sco_map[i].isoch_rx_q.sco_rx_pkt = NULL;

            bz_isoch_conns++;

            bz_isoch_sco_map[i].sco_tx_entry_manager.header = NULL;
            bz_isoch_sco_map[i].sco_tx_entry_manager.ender = NULL;        
            bz_isoch_sco_map[i].sco_tx_entry_manager.pkt_num = 0;
            bz_isoch_sco_map[i].sco_tx_entry_manager.wait_head = NULL;
            bz_isoch_sco_map[i].sco_tx_entry_manager.wait_end = NULL;        
            bz_isoch_sco_map[i].sco_tx_entry_manager.offset = 0;         

#ifdef _SCO_SEND_SINE_TONE_PKT_
            bz_isoch_sco_map[i].sco_tx_entry_manager.sine_tx_index = 0;
#endif

            bz_isoch_sco_map[i].next_instant = 0xFFFFFFFF;

            /* add by austin to program voice data capabilities */
            if (lmp_ce->is_sco_channel)
            {
                bz_isoch_sco_map[i].sco_ch_id = lmp_ce->sco_ce_idx;
            }
            else if (lmp_ce->is_esco_channel)
            {
                bz_isoch_sco_map[i].sco_ch_id = lmp_ce->esco_ce_idx;
            }
            bz_isoch_sco_map[i].host_data_8bit = lmp_ce->txfifo_in_8bit_code;
            bz_isoch_sco_map[i].air_data_8bit = lmp_ce->rxfifo_in_8bit_code;
            
            if (lmp_ce->trx_codec_conv_type == BZDMA_CODEC_TX_CONV_TYPE_BYPASS)
            {
                bz_isoch_sco_map[i].no_transparent = 0;
            }
            else
            {
                bz_isoch_sco_map[i].no_transparent = 1;
            }

            bz_isoch_sco_map[i].active = TRUE;

            bz_sco_pkt_size  = ISOCH_SINGLE_RX_PKT_LENGTH * bz_isoch_conns;

            if( ( bz_isoch_sco_map[i].no_transparent)&&
                (!bz_isoch_sco_map[i].host_data_8bit))
            {
                bz_sco_pkt_size <<= 1;
            }

#ifdef _ENABLE_8821_HW_SCO_DEFAULT_TX_DATA_
#ifndef _FIX_ESCO_RETRANSMIT_PAYLOAD_CONTENT_
            if (!IS_USE_FOR_BQB)
#endif
            {
                /* Set SCO default tx data content when SCO TX FIFO is empty */
                UINT16 temp;

                if (lmp_ce->trx_codec_conv_type >= BZDMA_CODEC_TX_CONV_TYPE_L2C)
                {
                    /* output CVSD encode */
                    BB_write_baseband_register(BB_SCO_DEFAULT_REG, 0xAAAA);
                }
                else
                {
                    /* do not output CVSD encode */
                    BB_write_baseband_register(BB_SCO_DEFAULT_REG, 0x0000);                
                }

                temp = BB_read_baseband_register(SYNC_FIFO_CONFIG_REGISTER);
                temp &= 0x3FFF;
                temp |= i << 14;
                BB_write_baseband_register(SYNC_FIFO_CONFIG_REGISTER, temp);   
            }
#endif

            pf_enable_usb_interface_sco_filter(i, TRUE, sync_conn_handle, 1);

            MINT_OS_EXIT_CRITICAL();

            RT_BT_LOG(GREEN, BZ_FW_ISOCH_0380_1, 8, 
                                      ce_index,
                                      bz_isoch_sco_map[i].sco_ch_id, 
                                      bz_isoch_sco_map[i].host_data_8bit, 
                                      bz_isoch_sco_map[i].air_data_8bit,
                                      bz_isoch_sco_map[i].no_transparent,
                                      bz_isoch_conns, 
                                      bz_isoch_sco_map[i].conn_handle, 
                                      sync_conn_handle);

            return &bz_isoch_sco_map[i];
        }
    }
    return NULL;
}


/**
 * Removes iso-queue for a SCO connection, and updates the isochronous
 * pkt length acording to number of connections.
 *
 * \param sync_conn_handle Connection handle of sco connection to be removed.
 *
 * \return API_SUCCESS if removal successful, else API_FAILURE.
 */
API_RESULT bz_isoch_remove_sco_queues(UINT16 sync_conn_handle, UINT8 ce_index)
{
    UCHAR i;
    SCO_ISOCH_MAP *map;
    UCHAR conns;
    
    if ((sync_conn_handle == INVALID_CONN_HANDLE) ||
        (bz_isoch_conns == 0) ||
        (ce_index == INVALID_CE_INDEX))
    {
        return API_FAILURE;
    }

    for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        map = &bz_isoch_sco_map[i];
        
        if (map->conn_handle == sync_conn_handle)
        {
            DEF_CRITICAL_SECTION_STORAGE;
            
            MINT_OS_ENTER_CRITICAL();

            map->conn_handle = INVALID_CONN_HANDLE;
            map->isoch_tx_q.sco_pkt_wr_ptr = 0;
            map->isoch_tx_q.sco_pkt_rd_ptr = 0;

            if ((map->isoch_rx_q.fragment_length != 0) ||
                (map->isoch_rx_q.sco_rx_pkt != NULL))  
            {  
                OS_FREE_BUFFER(synchronous_data_to_host_pool_id, 
					    (void*)map->isoch_rx_q.sco_rx_pkt);
            }
          
            bz_isoch_conns--;

            bz_sco_pkt_size  = ISOCH_SINGLE_RX_PKT_LENGTH * bz_isoch_conns;

            if( ( map->no_transparent)&&
                (!map->host_data_8bit))
            {
                bz_sco_pkt_size <<= 1;
            }
			
            if(bz_sco_pkt_size == 0)
            {
                bz_sco_pkt_size = 48;
            }			
          
            map->isoch_rx_q.fragment_length = 0;
            map->isoch_rx_q.fragment_status = ERR_CORRECT_DATA;
            map->isoch_rx_q.sco_rx_pkt = NULL;
            map->sco_ch_id = 0;
            map->host_data_8bit = 0;
            map->air_data_8bit = 0;
            map->no_transparent = 0;
            map->active = FALSE;

            /* free any queued pending pkt */
            SCO_TX_ENTRY_TYPE *ent_cur;
            SCO_TX_ENTRY_TYPE *ent_next;

            ent_cur = map->sco_tx_entry_manager.header;

            if (ent_cur != NULL)
            {
                while (ent_cur != NULL)
                {
                    ent_next = ent_cur->next; 
                    
                    dma_tx_fifo_pkt_free((void * )ent_cur->free_addr,
                                          HCI_TRANSPORT_SYNC_DATA_PKT_TYPE); 
                    ent_cur = ent_next;
                }
                
                if (sco_tx_entry_free != NULL)
                {
                    map->sco_tx_entry_manager.ender->next = sco_tx_entry_free;                
                }
                sco_tx_entry_free = map->sco_tx_entry_manager.header;
            }

            ent_cur = map->sco_tx_entry_manager.wait_head;

            if (ent_cur != NULL)
            {
                while (ent_cur != NULL)
                {
                    ent_next = ent_cur->next; 
                    
                    dma_tx_fifo_pkt_free((void * )ent_cur->free_addr,
                                           HCI_TRANSPORT_SYNC_DATA_PKT_TYPE); 
                    ent_cur = ent_next;
                }
                
                if (sco_tx_entry_free != NULL)
                {
                    map->sco_tx_entry_manager.wait_end->next = sco_tx_entry_free;                
                }
                sco_tx_entry_free = map->sco_tx_entry_manager.wait_head;
            } 
            
            map->sco_tx_entry_manager.header = NULL;
            map->sco_tx_entry_manager.ender = NULL;        
            map->sco_tx_entry_manager.pkt_num = 0;
            map->sco_tx_entry_manager.wait_head = NULL;
            map->sco_tx_entry_manager.wait_end = NULL;        
            map->sco_tx_entry_manager.offset = 0;

            /* free eSCO pending packets -- added by austin on 140106 */
            if (lmp_connection_entity[ce_index].is_esco_channel)
            {
                LMP_ESCO_DATA_Q *esco_data_q_ptr;
                UINT8 esco_ce_idx = lmp_connection_entity[ce_index].esco_ce_idx;
                
                esco_data_q_ptr = &(lmp_esco_connection_entity[esco_ce_idx].esco_data_q);

                /* free pending esco packets */                
                UINT8 read_index = esco_data_q_ptr->read_index;
                UINT8 pending_length = esco_data_q_ptr->pending_length;
                while (pending_length)
                {
                    dma_tx_fifo_pkt_free(
                        (void * )esco_data_q_ptr->esco_data_pkt_Q[read_index],
                        HCI_TRANSPORT_SYNC_DATA_PKT_TYPE); 

                    read_index++;
                    
                    if (read_index >= MAX_ESCO_DATA_Q_LENGTH)
                    {
                        read_index = 0;
                    }  
                    pending_length--;
                }

                /* free waiting esco packets */
                lc_handle_esco_free_pkts_callback_new(esco_ce_idx);

                /* reset esco data queue */
                memset(&lmp_esco_connection_entity[i].esco_data_q, 0, 
                        sizeof(LMP_ESCO_DATA_Q));
            }

            pf_enable_usb_interface_sco_filter(i, TRUE, 0xFF, 1);

            conns = bz_isoch_conns;

            MINT_OS_EXIT_CRITICAL();

            RT_BT_LOG(GREEN, MSG_REM_BZ_FW_ISOCH, 3, 
                                    conns, i, sync_conn_handle);
            
            return API_SUCCESS;
        }
    }
    return API_FAILURE;
}

/**
 * Writes data in tx iso-queue of a SCO connection to it's corresponding
 * hardware fifo.
 *
 * \param sco_ce_ptr. Pointer to sco connection entity structure.
 *                    Set by caller.
 *
 * \return API_SUCCESS if successful write to fifo, else API_FAILURE in
 *         the case of underflow of iso-queue.
 */
SECTION_ISR  API_RESULT bz_isoch_write_sco_tx_data(LMP_SCO_CONNECTION_DATA* sco_ce_ptr)
{
    UINT16 conn_handle;
    UINT16 length;
    ISOCH_SCO_TX_QUEUE* txq;
    UINT8 tx_id;  
    BOOLEAN result = FALSE;
    UINT8 segm_index = 0;
    UINT8 i = 0;
    UINT16 total_len = 0;
    ISOCH_SCO_TX_QUEUE_INFO qinfo;
    BZDMA_TX_DESC_ENTRY_STATUS *pTxEnt; 
    SCO_ISOCH_MAP *sco_map;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 cur_clk;
    UINT8 exit = FALSE;
    UINT8 lag = FALSE;
    UINT32 next_instant;
    UINT8 cnt = 0;    
    UINT16 recv_len;
    UINT32 diff;        
    BZDMA_TX_DESC_SEGMENT tx_desc;
    UINT8 j = 0;

    if (sco_ce_ptr == NULL)
    {
        return API_FAILURE;
    }

    conn_handle = sco_ce_ptr->sco_conn_handle;
    length = sco_ce_ptr->pkt_length;
    
    ce_ptr = &lmp_connection_entity[sco_ce_ptr->conn_entity_index];

    txq = (ISOCH_SCO_TX_QUEUE*)pf_hci_transport_isoch_get_handle(conn_handle);

    if (txq == NULL)
    {
        return API_FAILURE;
    }

    sco_map = (SCO_ISOCH_MAP *)txq->scoch_map;   

    if (sco_map->no_transparent)
    {
        /* this means the voice data needs to convert via codec */
        if (!sco_map->host_data_8bit)
        {
            /* double data length to meet constant rate */
            length <<= 1;
        }
    } 

    recv_len = length;

#ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_    
    tx_id = bzdma_get_dedicated_free_tx_entry(BZDMA_TX_ENTRY_TYPE_SCO); 
#else
    tx_id = bzdma_get_dedicated_free_tx_entry(BZDMA_TX_ENTRY_TYPE_NEW_SCO0); 
#endif

#if defined(_NEW_BZDMA_FROM_V7_) && defined(BZDMA_USE_NEW_SCO_TXCMD)
    /* quit the loop until next_instant is early than current clock */
    while (exit != TRUE)
    {
        if (sco_map->sco_tx_entry_manager.header != NULL)
        {
            /* has pending packet in the queue */
            cnt++;    

            if (recv_len != length)
            {
                recv_len = length;
            }
            
            result = pf_hci_transport_degueue_tx_pkt(
                            sco_map, &recv_len, &segm_index, &qinfo, FALSE);
        }        
    }
#else

    /* quit the loop until next_instant is early than current clock */
    while (exit != TRUE)
    {        
        lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

#ifndef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_
        /* dape modified. To avoid long-time sco connection error. 
            Not to compare bit27. */
        cur_clk = cur_clk & 0x03FFFFFF;
        next_instant = (sco_map->next_instant) & 0x03FFFFFF;
        ////////////////////////////////////////////////
#else
        next_instant = sco_map->next_instant;
#endif

        diff = lc_get_clock_diff(next_instant, cur_clk, 0);

        if (diff & BIT27)
        {
            lag = TRUE;            
        }
        else
        {    
            lag = FALSE;
            exit = TRUE;
        }

        if (sco_map->sco_tx_entry_manager.header != NULL)
        {
            /* has pending packet in the queue */
            cnt++;    

            if (recv_len != length)
            {
                recv_len = length;
            }
            
            result = pf_hci_transport_degueue_tx_pkt(
                                sco_map, &recv_len, &segm_index, &qinfo, lag);
        }
#ifdef _FAST_SCO_ESCO_RE_SYNC_ANCHOR_POINT_        
        else if (lag)
        {          
            sco_map->next_instant = cur_clk + (sco_ce_ptr->Tsco << 1);
            sco_map->next_instant &= 0x0FFFFFFF;
            break;
        }
#endif

        sco_map->next_instant += (sco_ce_ptr->Tsco << 1);
        sco_map->next_instant &= 0x0FFFFFFF;
    }
#endif
    
    if (tx_id == BZDMA_TX_DESC_ENTRY_NUM)
    {              
        /* no free tx entry. we can free any packet in waiting queue */
        pf_hci_transport_free_tx_buf(conn_handle);        
        return API_FAILURE;
    }

    pTxEnt = &Bzdma_Manager.TxEntSta[tx_id];

    if ((result == FALSE) && (cnt > 0))
    {
        /* need to free sco tx command */            
        bzdma_release_tx_entry(tx_id);

        /* Return failure and consequently transmit null for
           empty isoch-q ie; No data received from host. */
        return API_FAILURE;
    }

    if (segm_index == 0)
    {
        /* no any valid packet to use */
        
#ifndef _SCO_IMPROVE_VOICE_OUT_QUALITY_
        /* need to free sco tx command */            
        bzdma_release_tx_entry(tx_id);

        return API_FAILURE;
#else
        recv_len = 0;
#endif        
    }

#ifdef _SCO_SEND_SINE_TONE_PKT_
#ifdef _SCO_SEND_PACKET_IN_A_FIXED_MEMORY_
    UINT8 offset;

    offset = sco_map->sco_tx_entry_manager.sine_tx_index;    
    pTxEnt->pTxDesc[0].DWord0 = 0;
    pTxEnt->pTxDesc[0].DWord1 = 0; 
    pTxEnt->pTxDesc[0].start_addr = 
                (UINT32)(&sco_map->sco_tx_entry_manager.tone_src[offset]); 
    pTxEnt->pTxDesc[0].len = length;    
    pTxEnt->pTxDesc[0].isLast = TRUE;
    pTxEnt->total_len = total_len;
    pTxEnt->is_sco_lt = TRUE;
    pTxEnt->ch_id = sco_ce_ptr->sco_number;

    /* send tx command */
    bzdma_send_txcmd(tx_id);

    offset = (offset + (length >> 1)) & sco_map->sco_tx_entry_manager.tone_repeat_mask;       
    sco_map->sco_tx_entry_manager.sine_tx_index = offset;

    return API_SUCCESS;
#endif
#endif
    
    if (recv_len != length)
    {    
        /* fill zero to the buffer - required max payload size is 60 Bytes */
        tx_desc.DWord0 = 0;
        tx_desc.DWord1 = 0;
        tx_desc.start_addr = (UINT32)sco_tx_zero_buf; 
        tx_desc.len = length - recv_len;   
        pTxEnt->pTxDesc[i].DWord0 = tx_desc.DWord0;
        pTxEnt->pTxDesc[i].DWord1 = tx_desc.DWord1; 
        total_len += tx_desc.len;
        i++;
        segm_index++;
    }
    
    for (; i<segm_index; i++)
    {
        /* reduce no D-Cache impact. we compose the tx descriptor in the dmem 
           first then copy to sram */
        tx_desc.DWord0 = 0;
        tx_desc.DWord1 = 0;
        tx_desc.start_addr = (UINT32)qinfo.seg_pkt_add[j]; 
        tx_desc.len = qinfo.seg_max_len[j];
        pTxEnt->pTxDesc[i].DWord0 = tx_desc.DWord0;
        pTxEnt->pTxDesc[i].DWord1 = tx_desc.DWord1; 
        total_len += qinfo.seg_max_len[j];
        j++;
    }

    pTxEnt->pTxDesc[segm_index-1].isLast = TRUE;
    pTxEnt->total_len = total_len;
    pTxEnt->is_sco_lt = TRUE;
    
    pTxEnt->ch_id = sco_ce_ptr->sco_number;

    /* send tx command */
    bzdma_send_txcmd(tx_id);

    return API_SUCCESS;
}


/**
 * Return isoch sco rx queue handle.
 *
 * \param sync_conn_handle. Connection handle of SCO connection.
 *
 * \return Isoch rx queue handle if connection is registered, else
 *         returns NULL.
 */
void* bz_isoch_get_rx_q(UINT16 sync_conn_handle)
{
    UCHAR i;

    for(i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        if(bz_isoch_sco_map[i].conn_handle == sync_conn_handle)
        {
            return (void*)(&(bz_isoch_sco_map[i].isoch_rx_q));
        }
    }
    return NULL;
}

#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_
void bz_isoch_free_all_packets_in_sub_rx_queue(UINT8 queue_id)
{
    ISOCH_SCO_HCI_RX_SUB_QUEUE *rxq;

    rxq = &bz_isoch_queue_manager.rxq[queue_id];

    while (rxq->rptr != rxq->wptr)
    {
        if (rxq->pend_pktcnt == 0)
        {
            /* warning !! */
            break;
        }
        
        OS_FREE_BUFFER(synchronous_data_to_host_pool_id, 
            (void*)rxq->ring_array[rxq->rptr]);     
        
        rxq->rptr = (rxq->rptr + 1) & (ISOCH_RX_QUEUE_MAX_PKTCNT - 1);
        rxq->pend_pktcnt--;
        bz_isoch_queue_manager.pend_pktcnt--;
    }    

    /* init the sub-queue */
    rxq->wptr = 0;
    rxq->rptr = 0;    
    rxq->pend_pktcnt = 0;  
    rxq->active = FALSE;
}

void bz_isoch_free_all_packets_in_rx_queue(void)
{
    UINT8 i;    

    if (bz_isoch_queue_manager.pend_pktcnt == 0)
    {
        return;
    }

    for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        bz_isoch_free_all_packets_in_sub_rx_queue(i);
        
        if (bz_isoch_queue_manager.pend_pktcnt == 0)
        {
            break;
        }
    }

    bz_isoch_queue_manager.cur_dequeue_ptr = 0;    
    bz_isoch_queue_manager.cur_enqueue_ptr = 0;
    bz_isoch_queue_manager.pend_pktcnt = 0;    
}

void bz_isoch_dequeue_and_send_data_to_host(void)
{
    HCI_SYNC_DATA_PKT *sync_data_pkt;
    ISOCH_SCO_HCI_RX_SUB_QUEUE *rxq;
#ifndef _DAPE_TEST_CHK_ESCO_DATA_RX
    if (bz_isoch_conns == 0)
#endif
    {
        /* free all pending packets */
        bz_isoch_free_all_packets_in_rx_queue();
        return;
    }
            
    while (1)
    {        
        if ((bz_isoch_conns == 0) || (bz_isoch_queue_manager.pend_pktcnt == 0))
        {
            /* this can avoid to enter infinite loop if active sco channels 
               are gone */
            break;
        }

        rxq = &bz_isoch_queue_manager.rxq[bz_isoch_queue_manager.cur_dequeue_ptr];     
        
        if (rxq->active)
        {
            if (rxq->pend_pktcnt == 0)
            {                
                /* TODO: check here */                
                break;
            }

            sync_data_pkt = (HCI_SYNC_DATA_PKT* )rxq->ring_array[rxq->rptr];
            
			hci_td_tx_packet(HCI_TRANSPORT_SYNC_DATA_PKT_TYPE,
				(UCHAR *)(sync_data_pkt), (sync_data_pkt->packet_length + 3));            

            rxq->rptr++;
            rxq->rptr &= (ISOCH_RX_QUEUE_MAX_PKTCNT - 1); 
            rxq->pend_pktcnt--;  
            bz_isoch_queue_manager.pend_pktcnt--;            
        }
        
        if (bz_isoch_queue_manager.cur_dequeue_ptr == (ISOCH_SCO_MAX_CONNS - 1))
        {
            bz_isoch_queue_manager.cur_dequeue_ptr = 0;
        }
        else
        {
            bz_isoch_queue_manager.cur_dequeue_ptr++;
        }            
    }
}

#endif

API_RESULT bz_isoch_send_data_to_host( UINT16 conn_handle,
        UCHAR erroneous_data_reporting, UCHAR fifo_num,
        UINT16 length, UCHAR data_status)
{
    HCI_SYNC_DATA_PKT *sync_data_pkt;
    UINT32 frag_length;
    UINT32 frag_status;
    ISOCH_SCO_RX_QUEUE* isoch_rx_queue;
    UINT32 host_pkt_length;
    UINT16 length_to_read = 0;
    UINT16 length_remaining = length;   /* required data length to host */ 
    SCO_ISOCH_MAP *sco_map;
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_	
    UINT8 en_queue_count = 0;
#endif

#ifdef _ROM_CODE_PATCHED_
#ifdef _BRUCE_8821B_PLC_RCP_
    UCHAR return_status;
    if (rcp_bz_isoch_send_data_to_host != NULL)
    {
        if (rcp_bz_isoch_send_data_to_host( (void *)&return_status,(void *)&conn_handle,
                                            (void *)&erroneous_data_reporting,
                                            (void *)&fifo_num,(void *)&length,
                                            (void *)&data_status))
        {
            return return_status;
        }     
    }
#endif
#endif



#ifdef TEST_MODE
    if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
    {
    	void* isoch_handle;
		isoch_handle = pf_hci_transport_isoch_get_handle(conn_handle);

        /* we only loopback correct packet to remote side (130605) - austin */
            
        if ((isoch_handle != NULL) && (data_status == ERR_CORRECT_DATA))
        {
            if (length > 0)
            {
                pf_hci_transport_isoch_dma_queue_rx_fifo_data(isoch_handle, length, conn_handle);
            }            
        }
        else
        {
            BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length);
        }
        return API_SUCCESS;
    }
#endif /* TEST_MODE */

    /* search connection handle to get rx queue of iso channel */
    isoch_rx_queue = bz_isoch_get_rx_q(conn_handle);

    if(isoch_rx_queue == NULL)
    {
        if(data_status != ERR_LOST_DATA)
        {
            BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length);
        }
        return API_FAILURE;
    }

    /* if the upstream bus is not ready, we drop downstream payload here - 
       austin */
    if (pf_hci_transport_is_isoch_ups_ready() == FALSE)
    {
        BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length);

        /* if rx queue has any pending hci synchronous pkt, we drop it now */
        if (isoch_rx_queue->sco_rx_pkt != NULL)
        {
            OS_FREE_BUFFER(synchronous_data_to_host_pool_id,
                           (void*)isoch_rx_queue->sco_rx_pkt);
            
            isoch_rx_queue->fragment_length = 0;
            isoch_rx_queue->fragment_status = ERR_CORRECT_DATA;            
            isoch_rx_queue->sco_rx_pkt = NULL; 
        }

#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_        
        bz_isoch_free_all_packets_in_rx_queue();
#endif
        
        return API_FAILURE;
    }

    sco_map = (SCO_ISOCH_MAP *)isoch_rx_queue->scoch_map;

    if (g_usb_iso_alternate_change[sco_map->sco_ch_id] == TRUE)
    {
        g_usb_iso_alternate_change[sco_map->sco_ch_id] = FALSE;
        
        /* if rx queue has any pending hci synchronous pkt, we drop it now */
        if (isoch_rx_queue->sco_rx_pkt != NULL)
        {
            OS_FREE_BUFFER(synchronous_data_to_host_pool_id,
                           (void*)isoch_rx_queue->sco_rx_pkt);
            isoch_rx_queue->fragment_length = 0;
            isoch_rx_queue->fragment_status = ERR_CORRECT_DATA;
            isoch_rx_queue->sco_rx_pkt = NULL; 
        }

#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_        
        bz_isoch_free_all_packets_in_sub_rx_queue(sco_map->sco_ch_id);
#endif
    }

    /* load current hci synchronous data pkt size (only data field) */
    host_pkt_length = bz_sco_pkt_size;

    if (host_pkt_length % 24)
    {
        RT_BT_LOG(RED, BZ_FW_ISOCH_424, 1, bz_sco_pkt_size);
    }
#ifndef _DAPE_TEST_CHK_ESCO_DATA_RX
    if ((IS_USE_FOR_BQB) || (IS_USE_FOR_MUTE))
    {
        /* check Erroneous Data Reporting bit of page 0 in LMP feature mask */
        if (!(lmp_feature_data.feat_page0[6] & ERRONEOUS_DATA_REPORTING))
        {
            /* If the frature of ED is not supported, 
               drop all SCO over HCI data */
            BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length);
            return API_SUCCESS; 
        }

        if (erroneous_data_reporting == 0x0)
        {
            /* If ED is not enabled, drop all SCO over HCI data.
               We do not send synchronous packet to BITE */
            BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length);
            return API_SUCCESS; 
        }

        if (!bqb_receive_sco_esco_pkt_in_ed_test)
        {
            if (data_status != ERR_CORRECT_DATA)
            {
                return API_SUCCESS;
            }
            bqb_receive_sco_esco_pkt_in_ed_test = TRUE;        
        }                    
    }
#endif
    frag_length = isoch_rx_queue->fragment_length;
    frag_status = isoch_rx_queue->fragment_status;
    sync_data_pkt = (HCI_SYNC_DATA_PKT *)(isoch_rx_queue->sco_rx_pkt);
    
    UINT8 double_dma_len = FALSE;   

    if (sco_map->no_transparent)
    {
        /* this means the voice data needs to convert via codec */
        if (!sco_map->host_data_8bit)
        {
            /* double data length to meet constant rate */
            double_dma_len = TRUE;
            length_remaining <<= 1;
        }
    }
        
    while(length_remaining > 0)
    {
        /* get new synchronous packet from pool */
        if(frag_length == 0)
        {
            if (OS_GET_FREE_BUFFERS(synchronous_data_to_host_pool_id) < 2)
            {
                /* near empty */
                if (data_status != ERR_LOST_DATA)
                {
                    if (double_dma_len)
                    {
                        /* half the length (it is actual length in the sco rx
                           fifo for flush) */
                        length_remaining >>= 1;
                    }
 #ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI                    
                    DMA_read_RXFIFO_and_flush_for_sync_link(length, TRUE, conn_handle);
 #else
                    BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length_remaining);
 #endif
                }      
    	        frag_status = ERR_CORRECT_DATA;              
                break;      
            }
            
            if (OS_ALLOC_BUFFER(synchronous_data_to_host_pool_id,
                            (void**)(&sync_data_pkt)) != BT_ERROR_OK)
            {
                if (data_status != ERR_LOST_DATA)
                {
                    if (double_dma_len)
                    {
                        /* half the length (it is actual length in the sco rx
                           fifo for flush) */
                        length_remaining >>= 1;
                    }
#ifdef _BRUCE_FLUSH_STALED_PKT_IN_ESCO_LINK_OVER_HCI 
                    DMA_read_RXFIFO_and_flush_for_sync_link(length, TRUE, conn_handle);
#else
                    BB_flush_baseband_SYNC_RX_FIFO(fifo_num, length_remaining);
#endif
                }
    	        frag_status = ERR_CORRECT_DATA;              
                break; 
            }
        }

        length_to_read = MIN(host_pkt_length - frag_length, length_remaining);

        if (data_status == ERR_LOST_DATA)
        {
            if (erroneous_data_reporting)
            {
                /* set partial HCI data to zero in missing (e)SCO payload part */
                memset(sync_data_pkt->hci_sync_data_packet + frag_length,
                    0x0, length_to_read);
            }
            else
            {
                // TODO: We can do any PLC here. We do zero-fill now

                /* set partial HCI data to zero in missing (e)SCO payload part */
                memset(sync_data_pkt->hci_sync_data_packet + frag_length,
                    0x0, length_to_read);

            }
        }
        else
        {
            /* move data from sco rx fifo in bluewiz to iso sync data pkt */
            BB_dma_read_baseband_SYNC_RX_FIFO(
                        sync_data_pkt->hci_sync_data_packet + frag_length, 
                        length_to_read, 
                        ((SCO_ISOCH_MAP*)isoch_rx_queue->scoch_map)->sco_ch_id, 
                        TRUE); 
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
#if 1
//if (frag_length == 0)
g_esco_print_log ++;
if ((g_esco_print_log % 500) == 0)
{//if (length_remaining == length)
{
    UINT16 ce_index;
    UINT32 cur_clk;
    UINT16 sync_ce_index;

    lmp_get_esco_ce_index_from_conn_handle(conn_handle, &sync_ce_index);
    ce_index = lmp_esco_connection_entity[sync_ce_index].ce_index;

	
    lc_get_clock_in_scatternet(&cur_clk, 
	                          lmp_connection_entity[ce_index].phy_piconet_id);
    RT_BT_LOG(WHITE, DAPE_TEST_LOG550, 14, cur_clk, ce_index,
	frag_length,data_status,
sync_data_pkt->hci_sync_data_packet[frag_length],            
sync_data_pkt->hci_sync_data_packet[frag_length+1], 
sync_data_pkt->hci_sync_data_packet[frag_length+2], 
sync_data_pkt->hci_sync_data_packet[frag_length+3], 
sync_data_pkt->hci_sync_data_packet[frag_length+4], 
sync_data_pkt->hci_sync_data_packet[frag_length+5],
sync_data_pkt->hci_sync_data_packet[frag_length+6],            
sync_data_pkt->hci_sync_data_packet[frag_length+7], 
sync_data_pkt->hci_sync_data_packet[frag_length+8],            
sync_data_pkt->hci_sync_data_packet[frag_length+9]);
}
}
#endif
#endif
        }
        
        if (erroneous_data_reporting == 0x0)
        {
            /* the controller shall set the packet_status_flag to 00 if the
               Erroneous_Data_Reporting parameter was set to disabled 
               - austin */
            frag_status = 0;
        }
        else
        {
            /* Set status for current fragment */
            if(frag_length == 0)
            {
                frag_status = data_status;
            }
            else if (frag_status != ERR_INVALID_DATA)
            {
                switch(data_status)
                {
                    case ERR_CORRECT_DATA:
                        if(frag_status != ERR_CORRECT_DATA)
                        {
                            frag_status = ERR_PARTIAL_DATA;
                        }
                        break;
                    case ERR_INVALID_DATA:
                        frag_status = data_status;
                        break;
                    case ERR_LOST_DATA:
                        if(frag_status != ERR_LOST_DATA)
                        {
                            frag_status = ERR_PARTIAL_DATA;
                        }
                        break;
                    case ERR_PARTIAL_DATA:
                        frag_status = ERR_PARTIAL_DATA;
                        break;
                    default :
                        BZ_ASSERT(FALSE, "Invalid status flag");
                        break;
                }
            } /* end of if(frag_length == 0) */
        }
        
        length_remaining -= length_to_read;
        frag_length += length_to_read;

        if (frag_length == host_pkt_length)
        {
            /* we can remove hci synchronous rx packet form the rx queue,
               so init the temp */
            isoch_rx_queue->sco_rx_pkt = NULL;
            isoch_rx_queue->fragment_length = 0;
            isoch_rx_queue->fragment_status = ERR_CORRECT_DATA;            

            /* fill the header of hci synchronous packet */
            sync_data_pkt->connection_handle = conn_handle;
            sync_data_pkt->data_status_flag = frag_status;
            sync_data_pkt->reserved = 0x0;
            sync_data_pkt->packet_length = host_pkt_length;
			
#ifdef _SCO_SEND_SINE_TONE_PKT_
            /* to generate 1KHz pure tone */
            static UINT8 index[3] = {0, 0, 0};
            UINT16 i;
            UINT16 *ptr;
            
            ptr = (UINT16*)sync_data_pkt->hci_sync_data_packet;
            for (i = 0; i < (host_pkt_length >> 1); i++)
            {
                ptr[i] = Bzdma_1KHz_Sine_16BIT_SAMPLE[index[sco_map->sco_ch_id]];
                index[sco_map->sco_ch_id]++;
                index[sco_map->sco_ch_id] &= 0x07;
            }
#endif
			
#ifndef _FIX_MULTIPLE_SCO_RECORD_PATH_			
            /* CHECK: we send HCI sco data pkt to host via HCI DMA in 0380, 
               so shall use the same API in varied HCI interface - austin */
			hci_td_tx_packet(HCI_TRANSPORT_SYNC_DATA_PKT_TYPE,
				(UCHAR *)(sync_data_pkt), (sync_data_pkt->packet_length + 2 + 1));
#else
            /* enqueue hci synchronous packet */
            ISOCH_SCO_HCI_RX_SUB_QUEUE *rxq;
            rxq = &bz_isoch_queue_manager.rxq[sco_map->cur_id];
            rxq->ring_array[rxq->wptr] = (UINT32)sync_data_pkt;
            rxq->wptr++;
            rxq->wptr &= (ISOCH_RX_QUEUE_MAX_PKTCNT - 1); 
            rxq->pend_pktcnt++;
            rxq->active = TRUE;
            bz_isoch_queue_manager.cur_enqueue_ptr = sco_map->cur_id;
            bz_isoch_queue_manager.pend_pktcnt++;
            en_queue_count++;
#endif
            frag_length = 0;
    	    frag_status = ERR_CORRECT_DATA;

            /* hci actual pkt is sent to host. we can reset the 
               pointer now - austin */
            sync_data_pkt = NULL;
        } /* end of if (frag_length == host_pkt_length) */
    } /* end of while(length_remaining > 0) */

    isoch_rx_queue->fragment_length = frag_length;
    isoch_rx_queue->fragment_status = frag_status;
    isoch_rx_queue->sco_rx_pkt = sync_data_pkt;   
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
#if 0
RT_BT_LOG(WHITE, DAPE_TEST_LOG550, 12, 
sync_data_pkt->hci_sync_data_packet[frag_length],            
sync_data_pkt->hci_sync_data_packet[frag_length+1], 
sync_data_pkt->hci_sync_data_packet[frag_length+2], 
sync_data_pkt->hci_sync_data_packet[frag_length+3], 
sync_data_pkt->hci_sync_data_packet[frag_length+4], 
sync_data_pkt->hci_sync_data_packet[frag_length+5],
sync_data_pkt->hci_sync_data_packet[frag_length+6],            
sync_data_pkt->hci_sync_data_packet[frag_length+7], 
sync_data_pkt->hci_sync_data_packet[frag_length+8], 
sync_data_pkt->hci_sync_data_packet[frag_length+9], 
sync_data_pkt->hci_sync_data_packet[frag_length+10], 
sync_data_pkt->hci_sync_data_packet[frag_length+11]);
#endif
#endif    
#ifdef _FIX_MULTIPLE_SCO_RECORD_PATH_	
    if (en_queue_count > 0)
    {
        /* dequeue hci synchronous packet */
        bz_isoch_dequeue_and_send_data_to_host();        
    }
#endif

    return API_SUCCESS;
}

#endif /* SCO_OVER_HCI */

