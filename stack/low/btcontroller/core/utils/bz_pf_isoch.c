enum { __FILE_NUM__= 112 };

#include "bz_pf_isoch.h"
#include "logger.h"
#include "UartPrintf.h"
#include "bt_fw_os.h"
#include "bzdma.h"
#include "bt_fw_globals.h"
#include "lmp_defines.h"
#include "mint_os_buffer_internal.h"
#include "bb_driver.h"
#include "dma_usb.h"
#include "mem.h"
#include "lmp_1_2_defines.h"
#include "lc_1_2_internal.h"
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX        
#include "lmp.h"
#include "lc_internal.h"
extern LMP_CONNECTION_ENTITY lmp_connection_entity[LMP_MAX_CE_DATABASE_ENTRIES];
extern LMP_ESCO_CONNECTION_ENTITY lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
extern UINT16 g_esco_print_log;

#endif
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern UINT8 g_gen_fake_esco_data;
#endif
/********************* Global Variable Definations ************************/

extern SCO_ISOCH_MAP bz_isoch_sco_map[ISOCH_SCO_MAX_CONNS];
extern volatile UCHAR bz_sco_pkt_size;
extern BOOLEAN bz_isoch_is_usb_ready;
extern POOL_ID                 synchronous_data_to_remote_id;
extern POOL_ID                 synchronous_data_to_host_pool_id;
extern LMP_SCO_CONNECTION_DATA lmp_sco_connection_data[LMP_MAX_SCO_CONN_ENTRIES];
extern OS_POOL_MGR             pool_mgr;
extern LMP_SELF_DEVICE_DATA    lmp_self_device_data;
extern SCO_TX_ENTRY_TYPE *sco_tx_entry_free;
extern SCO_TX_ENTRY_TYPE sco_tx_entry_unit[BT_FW_TOTAL_SYNCHRONOUS_TX_ENTRY];
#ifdef _SCO_SEND_SINE_TONE_PKT_
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
extern UINT32 sco_num_in_tx_pkt;
#endif
#endif
#ifdef _DAPE_TEST_CHECK_SCO_TX_INDEX
extern UINT32 dape_sco_tx_index;
#endif
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
extern void gen_esco_data_packet(UINT8 change);
extern UINT16 g_esco_print_log;

#endif

/**
 * Function     : pf_hci_transport_isoch_get_pkt_length
 *
 * Description  : This function is used to get current sco packet size
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
*/
INLINE UCHAR pf_hci_transport_isoch_get_pkt_length(void)
{
	return bz_sco_pkt_size;
}


INLINE UINT8 pf_hci_transport_is_isoch_ups_ready(void)
{
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX
        return TRUE;        
#else
#ifndef _REMOVE_HCI_PCIE_
        if ((g_fun_interface_info.b.bt_interface == PCIE_INTERFACE)||
           (g_fun_interface_info.b.bt_interface == USB_INTERFACE))
#else
        if(g_fun_interface_info.b.bt_interface == USB_INTERFACE)
#endif
        {
            return bz_isoch_is_usb_ready;
        }
        else
        {      
            return TRUE;        
        }

#endif    
}


/**
 * Function     : pf_hci_transport_isoch_get_handle
 *
 * Description  : This function is used to get sco tx queue point
 *
 * Parameters   : sync_conn_handle: sco connection handle
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
*/
void* pf_hci_transport_isoch_get_handle(UINT16 sync_conn_handle)
{
	UCHAR i;

	for(i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
	{
		if(bz_isoch_sco_map[i].conn_handle == sync_conn_handle)
		{
			return (void*)(&(bz_isoch_sco_map[i].isoch_tx_q));
		}
	}
	return NULL;
}

/**
 * Function     : pf_hci_transport_isoch_get_tx_entry
 *
 * Description  : This function is used to get tx entry manager of sco connection
 *
 * Parameters   : sync_conn_handle: sco connection handle
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
*/
void* pf_hci_transport_isoch_get_tx_entry(UINT16 sync_conn_handle)
{
    UCHAR i;

    for(i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        if(bz_isoch_sco_map[i].conn_handle == sync_conn_handle)
        {
            return (void*)&bz_isoch_sco_map[i].sco_tx_entry_manager;
        }
    }
    return NULL;
}

void pf_hci_init_free_sco_tx_entry(void)
{
    UINT8 i;
    
    memset(sco_tx_entry_unit, 0, sizeof(SCO_TX_ENTRY_TYPE) * BT_FW_TOTAL_SYNCHRONOUS_TX_ENTRY);
    for (i = 0; i < (BT_FW_TOTAL_SYNCHRONOUS_TX_ENTRY - 1); i++)
    {
        sco_tx_entry_unit[i].next = &sco_tx_entry_unit[i + 1];
    }
    sco_tx_entry_unit[i].next = NULL;
    sco_tx_entry_free = sco_tx_entry_unit;  
}

/**
 * Function     : pf_hci_transport_isoch_queue_pkt
 *
 * Description  : This function is used to queue hci sco packet to link list (sco_tx_entry_manager)
 *
 * Parameters   : synchronous_pkt: sco packet point
 *                        sync_conn_handle: sco connection handle
 *                        buffer: sco payload point
 *                        length: sco packet length
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
*/
BOOLEAN pf_hci_transport_isoch_queue_pkt(void *pkt)
{   
    SCO_TX_ENTRY_MANAGER_TYPE   *sco_tx_entry_manger;
    SCO_TX_ENTRY_TYPE *sco_tx_entry;
	HCI_SYNC_DATA_PKT *sco_pkt = (HCI_SYNC_DATA_PKT *)pkt;
    
    DEF_CRITICAL_SECTION_STORAGE;
        
    sco_tx_entry_manger = 
            pf_hci_transport_isoch_get_tx_entry(sco_pkt->connection_handle);
    
    if (sco_tx_entry_manger == NULL || sco_pkt == NULL)
    {   
        RT_BT_LOG(GRAY, BZ_PF_ISOCH_78, 0, 0);
        return FALSE;
    }

    MINT_OS_ENTER_CRITICAL();

    if (sco_tx_entry_free == NULL)
    {
        RT_BT_LOG(RED, DMA_ALLOCATE_ERROR, 0, 0);
    
        MINT_OS_EXIT_CRITICAL();        
        return FALSE;
    }

    sco_tx_entry = sco_tx_entry_free;
    sco_tx_entry_free = sco_tx_entry_free->next;
	
#ifdef _SCO_SEND_SINE_TONE_PKT_
#ifndef _SCO_SEND_PACKET_IN_A_FIXED_MEMORY_
    UINT8 i;    
    UINT8 mask;
    UINT16 *ptr;
    ptr = (UINT16*)sco_pkt->hci_sync_data_packet;  
    
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
    sco_tx_entry->start_index = sco_tx_entry_manger->sine_tx_index;
    sco_tx_entry->cmp_index = sco_tx_entry_manger->sine_tx_index;
    sco_tx_entry->count++;
#endif    

    mask = sco_tx_entry_manger->tone_repeat_mask;
    for (i = 0; i < (sco_pkt->packet_length >> 1); i++)
    {
        ptr[i] = sco_tx_entry_manger->tone_src[sco_tx_entry_manger->sine_tx_index];
        sco_tx_entry_manger->sine_tx_index++;
        sco_tx_entry_manger->sine_tx_index &= mask;        
    }    
#endif
#endif
    
    sco_tx_entry->sco_payload = sco_pkt->hci_sync_data_packet;
    sco_tx_entry->free_addr = (UINT32) sco_pkt;
    sco_tx_entry->length = sco_pkt->packet_length;
    sco_tx_entry->remain_len = sco_pkt->packet_length;
    sco_tx_entry->next = NULL;
	
    if (sco_tx_entry_manger->header == NULL)
    {
        sco_tx_entry_manger->header = sco_tx_entry;
    }
    else
    {
        sco_tx_entry_manger->ender->next = sco_tx_entry;
    }
    sco_tx_entry_manger->ender = sco_tx_entry;
    sco_tx_entry_manger->pkt_num++;  

    MINT_OS_EXIT_CRITICAL();

    return TRUE;
}


/**
 * Function     : pf_hci_transport_degueue_tx_pkt
 *
 * Description  : This function is used to dequeue hci sco packet from link list 
 *                     (sco_tx_entry_manager) and set input parameter of bzdam.
 *
 * Parameters   : sync_conn_handle: sco connection handle
 *                        tx_length: sco packet length (HV1, HV2, HV3)
 *                        *segm_index: bzdma segment number
 *                        *pInfo: bzdma entry parameter
 *
 * Returns      : OK
 *
 * Side Effects : None
 *
*/
SECTION_ISR BOOLEAN pf_hci_transport_degueue_tx_pkt(
                SCO_ISOCH_MAP *iso_ch_map, 
                UINT16 *tx_length, UINT8 *segm_index, 
                ISOCH_SCO_TX_QUEUE_INFO *pInfo, UINT8 lag)
{
    UINT16 sent_pkt_length = *tx_length;
    UINT16 segm_len = 0;
    SCO_TX_ENTRY_MANAGER_TYPE *sco_tx_entry_manger;
    SCO_TX_ENTRY_TYPE *sco_tx_entry;
    UINT32 total_length = 0;
    UINT16 offset;
    //DEF_CRITICAL_SECTION_STORAGE;    

    *segm_index = 0;
    
    if ((sent_pkt_length == 0) || (iso_ch_map == NULL))
    {
        return FALSE;
    }

    sco_tx_entry_manger = &iso_ch_map->sco_tx_entry_manager;

    //MINT_OS_ENTER_CRITICAL();
    
    sco_tx_entry = sco_tx_entry_manger->header;

    while (sco_tx_entry != NULL)
    {       
        offset = sco_tx_entry_manger->offset;
        
        if (sent_pkt_length >= sco_tx_entry->remain_len)
        {
            sent_pkt_length -= sco_tx_entry->remain_len;                        
            segm_len = sco_tx_entry->remain_len;
            sco_tx_entry->remain_len = 0;
            sco_tx_entry_manger->offset = 0;

            /* dequeue one entry from pending list of sco_tx_entry_manger */
            if (sco_tx_entry->next == NULL)
            {
                sco_tx_entry_manger->header = NULL;
                sco_tx_entry_manger->ender = NULL;
            }
            else
            {
                sco_tx_entry_manger->header = sco_tx_entry->next;
                sco_tx_entry->next = NULL;
            }

            /* enqueue one entry to waiting list of sco_tx_entry_manger */
            if (sco_tx_entry_manger->wait_head == NULL)
            {
                sco_tx_entry_manger->wait_head = sco_tx_entry;
            }
            else
            {
                sco_tx_entry_manger->wait_end->next = sco_tx_entry;
            }
            sco_tx_entry_manger->wait_end = sco_tx_entry;               
        }
        else
        {
            segm_len = sent_pkt_length;                
            sco_tx_entry->remain_len -= sent_pkt_length;
            sent_pkt_length = 0;
            sco_tx_entry_manger->offset += segm_len;
        }

        total_length += segm_len;

        if (lag == FALSE)
        {
            pInfo->seg_max_len[(*segm_index)] = segm_len;
            pInfo->seg_pkt_add[(*segm_index)] = (UINT32) &sco_tx_entry->sco_payload[offset];
        }  

#ifndef _SCO_SEND_PACKET_IN_A_FIXED_MEMORY_
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
        UINT8 k;
        UINT16 *ptr; 
        UINT16 err_count = 0;
               
        ptr = (UINT16*)&sco_tx_entry->sco_payload[offset];        
  
        for (k = 0; k < (segm_len >> 1); k++)
        {
            if (sco_tx_entry_manger->tone_src[sco_tx_entry->cmp_index] != ptr[k])
            {
                if (err_count < 3)
                {
                    RT_BT_LOG(RED, DAPE_TEST_LOG436, 10,
		              k, sco_tx_entry_manger->tone_src[sco_tx_entry->cmp_index],
		              ptr[k], 
                         sco_tx_entry_manger->tone_src[(sco_tx_entry->start_index + (offset >> 1) + k) & sco_tx_entry_manger->tone_repeat_mask],
                              &ptr[k],
                              sco_tx_entry->count, 
                              sco_tx_entry->sco_payload,
                              (UINT32)sco_tx_entry,
                              sco_num_in_tx_pkt, 
                              dape_sco_tx_index);
                }			
                err_count++;
            }
            sco_tx_entry->cmp_index++;
            sco_tx_entry->cmp_index &= sco_tx_entry_manger->tone_repeat_mask;
        }
#endif
#endif

        (*segm_index)++;  

        if (sent_pkt_length == 0)
        {
            break;
        }

        sco_tx_entry = sco_tx_entry_manger->header;            
    }

    *tx_length = total_length;

    //MINT_OS_EXIT_CRITICAL();
    
//    RT_BT_LOG(RED,SEGMA_NUM,1,(*segm_index));

    if (lag == TRUE)
    {
        if (total_length > 0)
        {
            pf_hci_transport_free_tx_buf(iso_ch_map->conn_handle);
        }
    }

    return TRUE;
}

/**
 * Function     : pf_hci_free_tx_pkt
 *
 * Description  : This function is used to free sco tx entry memory and free HCI DMA tx entry
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 * Side Effects : None
 *
*/
void pf_hci_free_tx_pkt(UINT8 sco_number)
{
    LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
    
    sco_ce_ptr = &lmp_sco_connection_data[sco_number];

    pf_hci_transport_free_tx_buf(sco_ce_ptr->sco_conn_handle);

}

/**
 * Function     : pf_hci_transport_free_tx_buf
 *
 * Description  : This function is used to free sco tx entry memory and free HCI DMA tx entry
 *
 * Parameters   : sync_conn_handle: sco connection handle
 *
 * Returns      : None
 *
 * Side Effects : None
 *
*/
BOOLEAN pf_hci_transport_free_tx_buf(UINT16 sync_conn_handle)
{
    SCO_TX_ENTRY_MANAGER_TYPE *sco_tx_entry_manger;
    SCO_TX_ENTRY_TYPE *sco_tx_entry;   
    SCO_TX_ENTRY_TYPE *sco_tx_entry_next;      
    DEF_CRITICAL_SECTION_STORAGE;  
       
    sco_tx_entry_manger = pf_hci_transport_isoch_get_tx_entry(sync_conn_handle);

    if (sco_tx_entry_manger == NULL)
    {
        RT_BT_LOG(GRAY, BZ_PF_ISOCH_78, 0, 0);
        return FALSE;
    }

    MINT_OS_ENTER_CRITICAL();    
    sco_tx_entry = sco_tx_entry_manger->wait_head;

    while (sco_tx_entry != NULL)
    {
        sco_tx_entry_next = sco_tx_entry->next;

#ifdef TEST_MODE
        if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
        {
            OS_FREE_BUFFER(synchronous_data_to_host_pool_id,
                                (void * )sco_tx_entry->free_addr);
        }     
        else
#endif            
        {
#ifdef _SCO_SEND_SINE_TONE_PKT_AND_COMPARE
            sco_tx_entry->count--;		
#endif        
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
            if (g_gen_fake_esco_data)
            {
                gen_esco_data_packet(FALSE);
            }
            else
            {
                dma_tx_fifo_pkt_free((void * )sco_tx_entry->free_addr,
                                       HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
            }
#else
            dma_tx_fifo_pkt_free((void * )sco_tx_entry->free_addr,
                                       HCI_TRANSPORT_SYNC_DATA_PKT_TYPE);
#endif                                       
        }
        
        sco_tx_entry = sco_tx_entry_next;

        if (sco_tx_entry_manger->pkt_num > 0)
        {
            sco_tx_entry_manger->pkt_num--;  
        }
    }

    if (sco_tx_entry_manger->wait_head != NULL)
    {
        sco_tx_entry_manger->wait_end->next = sco_tx_entry_free;
        sco_tx_entry_free = sco_tx_entry_manger->wait_head;

        sco_tx_entry_manger->wait_head = NULL;
        sco_tx_entry_manger->wait_end = NULL;          
    }
    MINT_OS_EXIT_CRITICAL();       

    return TRUE;
}


void pf_hci_transport_reset_isoch_queue(void)
{
    UINT8 i;

    /* reset sco tx entry */
    pf_hci_init_free_sco_tx_entry();

    for (i = 0; i < ISOCH_SCO_MAX_CONNS; i++)
    {
        /* init sco map in each valid sco channel */
        if (bz_isoch_sco_map[i].conn_handle != INVALID_CONN_HANDLE)
        {                
            bz_isoch_sco_map[i].sco_tx_entry_manager.header = NULL;
            bz_isoch_sco_map[i].sco_tx_entry_manager.ender = NULL;                
            bz_isoch_sco_map[i].sco_tx_entry_manager.wait_head = NULL;
            bz_isoch_sco_map[i].sco_tx_entry_manager.wait_end = NULL; 
            bz_isoch_sco_map[i].sco_tx_entry_manager.offset = 0;
            bz_isoch_sco_map[i].sco_tx_entry_manager.pkt_num = 0;

#ifdef _SCO_SEND_SINE_TONE_PKT_
            bz_isoch_sco_map[i].sco_tx_entry_manager.sine_tx_index = 0;
#endif
        }
    }

    clear_sco_buffer();        
}


/**
 * Function     : pf_hci_transport_isoch_dma_queue_rx_fifo_data
 *
 * Description  : This function is used to queue rx packet to tx queue list 
 *                for REMOTE LOOPBACK TEST
 *
 * Parameters   : isoch_handle: sco tx queue point
 *                length : receive sco packet length
 *                con_handle : sco connection handle
 *
 * Returns      : None
 *
 * Side Effects : None
 *
*/
BOOLEAN pf_hci_transport_isoch_dma_queue_rx_fifo_data(void* isoch_handle, 
                                                 UINT16 length, UINT16 con_handle)
{
    BZDMA_RX_DESC_SEGMENT rxdesc[2];
    ISOCH_SCO_TX_QUEUE* txq = (ISOCH_SCO_TX_QUEUE*)isoch_handle;
    SCO_ISOCH_MAP *sco_map;
    HCI_SYNC_DATA_PKT *sync_data_pkt = NULL;

    if (OS_ALLOC_BUFFER(synchronous_data_to_host_pool_id,
                        (void**)(&sync_data_pkt)) != BT_ERROR_OK)
    {
        BB_flush_baseband_SYNC_RX_FIFO(0, length);
        return FALSE;
    }

    /* Note: the length field is the encode data from the air in the rxfifo */
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
        
	sync_data_pkt->connection_handle = con_handle;
	sync_data_pkt->packet_length = length;
    
	rxdesc[0].addr = (UINT32)&sync_data_pkt->hci_sync_data_packet[0];							  
    rxdesc[0].len = length;

    bzdma_send_burst_rxcmd_and_wait_complete(rxdesc, 1, BZDMA_RX_PID_SCO, 
                            ((SCO_ISOCH_MAP*)txq->scoch_map)->sco_ch_id, TRUE);

#ifdef COMPILE_ESCO
    UINT16 sync_ce_index;

    /* Check the Connection Handle is eSCO link or not */
    if (lmp_get_esco_ce_index_from_conn_handle(con_handle,
            &sync_ce_index) == API_SUCCESS)
    {
#ifdef _DAPE_TEST_CHK_ESCO_DATA_RX        
        if (lmp_self_device_data.test_mode == HCI_REMOTE_LOOPBACK_MODE)
        {
            //if (frag_length == 0)
            g_esco_print_log ++;
            if ((g_esco_print_log % 500) == 0)
            {	
                UINT16 ce_index;
                UINT32 cur_clk;
                UINT16 sync_ce_index;
            
                lmp_get_esco_ce_index_from_conn_handle(con_handle, &sync_ce_index);
                ce_index = lmp_esco_connection_entity[sync_ce_index].ce_index;

                lc_get_clock_in_scatternet(&cur_clk, 
                	       lmp_connection_entity[ce_index].phy_piconet_id);
            
                RT_BT_LOG(YELLOW, DAPE_TEST_LOG550, 14, cur_clk, 
                	ce_index, length,
                sync_data_pkt->hci_sync_data_packet[0],            
                sync_data_pkt->hci_sync_data_packet[1], 
                sync_data_pkt->hci_sync_data_packet[2], 
                sync_data_pkt->hci_sync_data_packet[3], 
                sync_data_pkt->hci_sync_data_packet[4], 
                sync_data_pkt->hci_sync_data_packet[5],
                sync_data_pkt->hci_sync_data_packet[6],            
                sync_data_pkt->hci_sync_data_packet[7], 
                sync_data_pkt->hci_sync_data_packet[8],            
                sync_data_pkt->hci_sync_data_packet[9],
                sync_data_pkt->hci_sync_data_packet[10]);
            }          
        }
#endif

        lc_queue_esco_data_pkt(sync_data_pkt, sync_ce_index);
        return TRUE;
    }
#endif /* COMPILE_ESCO */        

    if (pf_hci_transport_isoch_queue_pkt((void*)sync_data_pkt) == TRUE)
    {
        return TRUE;
    }

    OS_FREE_BUFFER(synchronous_data_to_host_pool_id, sync_data_pkt);

    return TRUE;
}


