/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 60 };
/********************************* Logger *************************/
#include "lmp_vendor_defines.h"
#include "lmp_vendor_internal.h"
#include "bt_fw_hci_internal.h"
#include "bt_fw_globals.h"
#include "hci_td.h"
#include "le_ll.h"
#include "mem.h"
#include "gpio.h"
#include "le_ll_driver.h"
#include "le_hci_4_0.h"

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_rssi_app_msft_update_adv_rssi_info = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rssi_app_msft_update_conn_rssi_info = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rssi_app_msft_check_rssi_monitor_condition = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rssi_app_msft_check_conn_thres_counter_then_send_event = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_rssi_app_msft_check_adv_counter_then_send_event = NULL;
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
UINT8 g_10ms_timer_tick_cnt = 0;
#endif

#if (defined(_LE_AUTO_REPORT_RSSI_AND_LOGIN_INOUT_FUNC_) || \
    defined(_SUPPORT_MSFT_BT_HCI_EXTENSION_)) && defined(LE_MODE_EN)
LL_CONN_HANDLE_RSSI_MANAGER ll_rssi_manager;

void rssi_app_le_send_hci_vendor_rssi_event(UINT16 handle, INT8 rssi)
{
    HCI_EVENT_PKT *hci_event_pkt_ptr;
    OS_SIGNAL signal_send;                
    
    if (OS_ALLOC_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                       (OS_ADDRESS *)&hci_event_pkt_ptr) == BT_ERROR_OK)
    {
        hci_event_pkt_ptr->event_opcode = HCI_VENDOR_SPECIFIC_EVENT;
        hci_event_pkt_ptr->param_total_length = 5;
        hci_event_pkt_ptr->event_parameter[0] = 0x22;
        hci_event_pkt_ptr->event_parameter[1] = 0x00;
        hci_event_pkt_ptr->event_parameter[2] = LSB(handle);
        hci_event_pkt_ptr->event_parameter[3] = MSB(handle);
        hci_event_pkt_ptr->event_parameter[4] = (UINT8)rssi;
        
        signal_send.type  = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
        signal_send.param = (OS_ADDRESS *)hci_event_pkt_ptr;
         
        /* Total length of the event packet = 
         * param_length (2nd  byte)+ Event type (1) + length field(1) */
         signal_send.length = 7;
        
        if (OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, 
                                   signal_send) != BT_ERROR_OK)
        {
            OS_FREE_BUFFER(tx_table[HCI_EVENT_HANDLER_TASK].pool_handle,
                           hci_event_pkt_ptr);          
        }
    }      
}

void rssi_app_le_bind_rssi_info(void)
{
    UINT8 i;
    UINT8 j;

    for (j = 0; j < ll_rssi_manager.max_indivadual_cnt; j++)
    {
        LL_CONN_HANDLE_RSSI_UNIT *punit;
        LL_CONN_REMOTE_INFO *prem;
            
        punit = &ll_rssi_manager.entry[j];
        ll_rssi_manager.bm_used_handle = 0;

        if (punit->addr_type == 2) /* BREDR type */
        {
            for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
            {
                LMP_CONNECTION_ENTITY *ce_ptr = &lmp_connection_entity[i];            
                if (ce_ptr->entity_status == ASSIGNED)
                {
                    if (!memcmp(ce_ptr->bd_addr, punit->addr, 6))
                    {
                        /* addrrss match !! */
                        ll_rssi_manager.bm_used_legacy_handle |= BIT(i);
                        ll_rssi_manager.legacy_handle[i] = j;                        

                        //RT_BT_LOG(GRAY, MSG_LE_BIND_RSSI_INFO, 2, i, j); 
                    }
                }
            }
        }
        else        
        {
            /* check possible le acl links */
            for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
            {
                if (ll_manager.conn_unit.handle[i].connected &&
                    (ll_manager.conn_unit.bmActiveHandle & (1 << i)))
                {
                    prem = &ll_manager.conn_unit.handle[i].remote_info;
                    if ((prem->addr_type == punit->addr_type) &&
                        !memcmp(prem->addr, punit->addr, 6))
                    {
                        /* addrrss match !! */
                        ll_rssi_manager.bm_used_handle |= BIT(i);
                        ll_rssi_manager.handle[i] = j;

                        //RT_BT_LOG(GRAY, MSG_LE_BIND_RSSI_INFO, 2, i, j);                    
                    }
                }
            }   
        }
    }           
}

UINT8 rssi_app_le_check_rssi_condition(UINT8 addr_type, UINT8 *rem_addr, INT8 rssi)
{
    UINT8 i;

    for (i = 0; i < ll_rssi_manager.max_indivadual_cnt; i++)
    {
        LL_CONN_HANDLE_RSSI_UNIT *punit = &ll_rssi_manager.entry[i];
        
        if ((addr_type == punit->addr_type) && 
                !memcmp(rem_addr, punit->addr, 6))
        {
            /* find matched address and type !! */
            
            if ((rssi <= punit->max_rssi_thres) &&
                    (rssi >= punit->min_rssi_thres))
            {
                /* send event to host */
                return LE_WAKEON_STATE_PASS_IN_RSSI_LIST;
            }

            return LE_WAKEON_STATE_FAIL_IN_RSSI_LIST;
        }
    }
        
    return LE_WAKEON_STATE_NOT_IN_RSSI_LIST;   
}

void rssi_app_vendor_check_and_send_rssi_event(
                                UINT8 mode, UINT8 index, 
                                UINT16 rssi_count, INT32 rssi_sum)
{
    INT8 rssi_adv;
    UINT8 entry_no = 0xff;
    UINT16 conn_handle = 0;
  
    if (rssi_count == 0)
    {
        rssi_adv = rssi_sum;
    }
    else
    {
        rssi_adv = rssi_sum / rssi_count;              
    }                 

    do
    {
        if (mode == 0)
        {
            if (!((ll_rssi_manager.bm_used_legacy_handle & (1 << index)) &&
                (lmp_connection_entity[index].entity_status == ASSIGNED)))
            {
                break;
            }
            conn_handle = 
                lmp_connection_entity[index].connection_type.connection_handle;
        }
        else       
        {
            if (!((ll_rssi_manager.bm_used_handle & (1 << index)) &&
                 ll_manager.conn_unit.handle[index].connected &&
                (ll_manager.conn_unit.bmActiveHandle & (1 << index))))        
            {
                break;                
            }
            conn_handle = ll_manager.conn_unit.handle[index].conn_handle;
        }  

        entry_no = ll_rssi_manager.handle[index];
        
        if ((rssi_adv >= ll_rssi_manager.entry[entry_no].min_rssi_thres) &&
            (rssi_adv <= ll_rssi_manager.entry[entry_no].max_rssi_thres))
        {
            rssi_app_le_send_hci_vendor_rssi_event(conn_handle, rssi_adv);
        } 
    }
    while (0);
    
    RT_BT_LOG(YELLOW, MSG_LE_RSSI_COUNT_INFO2, 6, 
                mode, entry_no, index, rssi_count, rssi_sum, rssi_adv);         
}
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
/**************************************************************************
 * Function     : rssi_app_msft_reset_le_adv_pattern
 *
 * Description  : This function is used to reset local pattern database 
 *                for le advertising monitor
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void rssi_app_msft_reset_le_adv_pattern(void)
{
    UINT8 i;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
#else
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT;
#endif

    for (i = 0; i < imax; i++)
    {
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        ll_rssi_manager.ms_adv_unit[i].next_id = i + 1;      
#else
        ll_rssi_manager.le_adv_pat[i].next_id = i + 1;                
#endif
    }
    ll_rssi_manager.le_adv_free_monitor_head = 0;
    ll_rssi_manager.le_adv_free_monitor_tail = (imax - 1); 
    ll_rssi_manager.le_adv_free_monitor_unit_cnt = imax;
    
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_    
    for (i = 0; i < 5; i++)
    {
        ll_rssi_manager.bm_le_adv_monitor_entry[i] = 0;
    }
#else
    ll_rssi_manager.bm_le_adv_monitor_entry = 0;
#endif
    ll_rssi_manager.le_adv_monitor_entry_count = 0;    
}

/**************************************************************************
 * Function     : rssi_app_msft_get_rssi_monitor_entry
 *
 * Description  : This function is used to get a rssi monitor entry for le 
 *                advertising monitor
 * Parameters   : type - condition type
 *                num_of_patterns - the num_of_patterns field of pattern data
 *                               (only valid if condition type is pattern data)  
 *
 * Returns      : handle_id  ( = 0xff means no resource)
 *
 *************************************************************************/
UINT8 rssi_app_msft_get_rssi_monitor_entry(UINT8 type, UINT8 num_of_patterns)
{
    UINT8 i;
    UINT8 handle_id = 0xff;
    UINT8 next;
    UINT8 cur = 0;
#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 max;
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    if (ll_rssi_manager.le_adv_monitor_entry_count == 0)
    {
        rssi_app_msft_reset_le_adv_pattern();
    }

    switch (type)
    {
    case LE_MSFT_MON_RSSI_FLAG_IRK:
    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
    case LE_MSFT_MON_RSSI_FLAG_UUID:
    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
        if (ll_rssi_manager.le_adv_free_monitor_head == LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT)
        {
            /* check the free list is empty or not ? */
            break;
        }
        
        if (ll_rssi_manager.le_adv_free_monitor_unit_cnt < num_of_patterns)
        {
            /* quit now if we no have enough free units or no 
               request any patterns */
            break;
        }

        /* cut the monitor units in the free list */
        handle_id = ll_rssi_manager.le_adv_free_monitor_head;
        next = handle_id;
        for (i = 0; i < num_of_patterns; i++)
        {
            cur = next;
            next = ll_rssi_manager.ms_adv_unit[next].next_id;
        }
        ll_rssi_manager.ms_adv_unit[cur].next_id = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;

        /* update the free monitor head and tail */
        if (next == LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT)
        {
            ll_rssi_manager.le_adv_free_monitor_head = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
            ll_rssi_manager.le_adv_free_monitor_tail = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
        }
        else
        {
            ll_rssi_manager.le_adv_free_monitor_head = next;
        }
        ll_rssi_manager.le_adv_free_monitor_unit_cnt -= num_of_patterns;
        ll_rssi_manager.le_adv_monitor_entry_count++;
        ll_rssi_manager.bm_le_adv_monitor_entry[0] |= (1 << handle_id);
        ll_rssi_manager.bm_le_adv_monitor_entry[type] |= (1 << handle_id);            
        ll_rssi_manager.le_adv_monitor_type |= 1 << type;
        return handle_id;
        
    default:
        break;
    }
#else
    switch (type)
    {
    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA: 
        if (ll_rssi_manager.le_adv_monitor_entry_count == 0)
        {
            rssi_app_msft_reset_le_adv_pattern();
            ll_rssi_manager.le_adv_monitor_type = type;
        }

        /* check the free list is empty or not ? */
        if (ll_rssi_manager.le_adv_free_monitor_head != LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT)
        {
            if ((ll_rssi_manager.le_adv_free_monitor_unit_cnt < num_of_patterns) ||
                (num_of_patterns == 0))
            {
                /* quit now if we no have enough free units or no 
                   request any patterns */
                return 0xff;
            }

            /* cut the monitor units in the free list */
            handle_id = ll_rssi_manager.le_adv_free_monitor_head;
            next = handle_id;
            for (i = 0; i < num_of_patterns; i++)
            {
                cur = next;
                next = ll_rssi_manager.le_adv_pat[next].next_id;
            }
            ll_rssi_manager.le_adv_pat[cur].next_id = LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT;

            /* update the free monitor head and tail */
            if (next == LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT)
            {
                ll_rssi_manager.le_adv_free_monitor_head = LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT;
                ll_rssi_manager.le_adv_free_monitor_tail = LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT;
            }
            else
            {
                ll_rssi_manager.le_adv_free_monitor_head = next;
            }
            ll_rssi_manager.le_adv_free_monitor_unit_cnt -= num_of_patterns;
            ll_rssi_manager.le_adv_monitor_entry_count++;
            ll_rssi_manager.bm_le_adv_monitor_entry |= (1 << handle_id);
            return handle_id;
        }
        else
        {
            return 0xff;
        }
        break;
        
    case LE_MSFT_MON_RSSI_FLAG_IRK:
    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
    case LE_MSFT_MON_RSSI_FLAG_UUID:
        if (ll_rssi_manager.le_adv_monitor_entry_count == 0)
        {
            ll_rssi_manager.bm_le_adv_monitor_entry = 0;
            ll_rssi_manager.le_adv_monitor_type = type;
        }

        if (type == LE_MSFT_MON_RSSI_FLAG_IRK)
        {
            max = LE_MSFT_MON_RSSI_MAX_SUPPORT_IRK_CNT; 
        }
        else if (type == LE_MSFT_MON_RSSI_FLAG_BDADDR)
        {
            max = LE_MSFT_MON_RSSI_MAX_SUPPORT_BD_ADDR_CNT;
        } 
        else
        {
            max = LE_MSFT_MON_RSSI_MAX_SUPPORT_UUID_CNT;
        }

        if (ll_rssi_manager.le_adv_monitor_entry_count < max)
        {
            for (i = 0; i < max; i++)
            {
                if (!(ll_rssi_manager.bm_le_adv_monitor_entry & (1 << i)))
                {
                    ll_rssi_manager.bm_le_adv_monitor_entry |= (1 << i);
                    ll_rssi_manager.le_adv_monitor_entry_count++; 
                    handle_id = i;
                    break;
                }
            }            
        } 
        break;
        
    default:
        return 0xff;
    }
#endif
    
    return handle_id;
}

/**************************************************************************
 * Function     : rssi_app_msft_free_rssi_monitor_entry
 *
 * Description  : This function is used to free a rssi monitor entry for le 
 *                advertising monitor
 * Parameters   : type - condition type
 *                hahandle_id - released id  
 *
 * Returns      : None
 *
 *************************************************************************/
void rssi_app_msft_free_rssi_monitor_entry(UINT8 type, UINT8 handle_id)
{
    UINT8 next = handle_id;
    UINT8 count = 0;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    switch (type)
    {
    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
    case LE_MSFT_MON_RSSI_FLAG_IRK:
    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
    case LE_MSFT_MON_RSSI_FLAG_UUID:            
        /* check current monitor type is valid or not */            
        while (ll_rssi_manager.ms_adv_unit[next].next_id != LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT)
        {
            count++;
            next = ll_rssi_manager.ms_adv_unit[next].next_id;
        }

        /* let free unit go back free list */
        if (ll_rssi_manager.le_adv_free_monitor_head == LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT)
        {
            ll_rssi_manager.le_adv_free_monitor_head = handle_id;
        }
        else
        {
            ll_rssi_manager.ms_adv_unit[ll_rssi_manager.le_adv_free_monitor_tail].next_id = handle_id;
        }
        ll_rssi_manager.le_adv_free_monitor_tail = next;
        ll_rssi_manager.le_adv_free_monitor_unit_cnt += count;
        break;
        
    default:
        return;
    }

    ll_rssi_manager.bm_le_adv_monitor_entry[0] &= ~(1 << handle_id);    
    ll_rssi_manager.bm_le_adv_monitor_entry[type] &= ~(1 << handle_id); 
    ll_rssi_manager.le_adv_monitor_entry_count--;
    ll_rssi_manager.adv_entry[handle_id].mon_state = LE_MSFT_MONITOR_RSSI_STATE_INIT;

    if (ll_rssi_manager.le_adv_monitor_entry_count == 0)
    {
        ll_rssi_manager.le_adv_monitor_type = LE_MSFT_MON_RSSI_FLAG_NONE;
    }

#else /* else of #ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ */

    switch (type)
    {
    case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
        /* check current monitor type is valid or not */            
        while (ll_rssi_manager.le_adv_pat[next].next_id != LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT)
        {
            count++;
            next = ll_rssi_manager.le_adv_pat[next].next_id;
        }

        /* let free unit go back free list */
        if (ll_rssi_manager.le_adv_free_monitor_head == LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT)
        {
            ll_rssi_manager.le_adv_free_monitor_head = handle_id;
        }
        else
        {
            ll_rssi_manager.le_adv_pat[ll_rssi_manager.le_adv_free_monitor_tail].next_id = handle_id;
        }
        ll_rssi_manager.le_adv_free_monitor_tail = next;
        ll_rssi_manager.le_adv_free_monitor_unit_cnt += count;
        break;
        
    case LE_MSFT_MON_RSSI_FLAG_IRK:
    case LE_MSFT_MON_RSSI_FLAG_BDADDR:
    case LE_MSFT_MON_RSSI_FLAG_UUID:            
        break;
        
    default:
        return;
    }

    ll_rssi_manager.bm_le_adv_monitor_entry &= ~(1 << handle_id);    
    ll_rssi_manager.le_adv_monitor_entry_count--;
    ll_rssi_manager.adv_entry[handle_id].mon_state = LE_MSFT_MONITOR_RSSI_STATE_INIT;

    if (ll_rssi_manager.le_adv_monitor_entry_count == 0)
    {
        ll_rssi_manager.le_adv_monitor_type = LE_MSFT_MON_RSSI_FLAG_NONE;
    }
#endif  /* end of #ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ */  
}

/**************************************************************************
 * Function     : rssi_app_msft_update_adv_rssi_info
 *
 * Description  : This function is used to update some information after get 
 *                a rssi sample (receive one adv packet).
 *                the information is include :
 *                (1) store rssi value
 *                (2) update monitor state machine after compare rssi
 *                (3) reset rssi threshold or monitor counter
 * 
 * Parameters   : handle_id - assigned handle id
 *                pHdr - the pointer of the head of advertising packet  
 *                rssi - the rssi value captured from this adv packet
 *
 * Returns      : None
 *
 *************************************************************************/
void rssi_app_msft_update_adv_rssi_info(UINT8 handle_id, UINT8 *pHdr, INT8 rssi)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
    LL_ADV_RSSI_UNIT *pentry = &ll_rssi_manager.adv_entry[handle_id];

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rssi_app_msft_update_adv_rssi_info != NULL)
    {
        if (rcp_rssi_app_msft_update_adv_rssi_info((void*)pentry, pHdr, rssi))
        {
            return;
        }
    }
#endif

    if (pentry->mon_state == LE_MSFT_MONITOR_RSSI_STATE_OUT_RANGE)
    {
        if ((rssi >= pentry->max_rssi_thres) && (pentry->notified == 0))
        {
            hci_generate_vs_msft_le_monitor_event(pRxPkt->Header.TxAdd, 
                                                pRxPkt->u1RemoteAddr,
                                                handle_id, 1);
            pentry->notified = 1;

            if ((pentry->sample_period != 0x00) && 
                (pentry->sample_period != 0xFF))
            {
                pentry->start_periodic_timer = TRUE;
                pentry->pkt_cnts = 0;
                pentry->pkt_rssi_sum = 0;                
            }
            
            pentry->mon_state = LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE;
        }
    }       
    else if (pentry->mon_state == LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE)
    {
        if (rssi > pentry->min_rssi_thres)
        {
            pentry->rssi_thres_counter = 0;
        }
            
        if (pentry->start_periodic_timer)
        {
            pentry->pkt_cnts++;
            pentry->pkt_rssi_sum += rssi;
        }        
    }
    
    /* store some info from received adv pdu */
    pentry->last_rssi = rssi;
    memcpy((UINT8*)&(pentry->loc_adv), pHdr, 
            LL_ADV_CH_PDU_HEADER_SIZE + pRxPkt->Header.Length);
}

/**************************************************************************
 * Function     : rssi_app_msft_update_conn_rssi_info
 *
 * Description  : This function is used to update some information after get 
 *                a rssi sample (receive a packet from BR/EDR or BLE link).
 *                the information is include :
 *                (1) store rssi value
 *                (2) update monitor state machine after compare rssi
 *                (3) reset rssi threshold or monitor counter
 *
 * Parameters   : handle_id - assigned handle id
 *                rssi - the rssi value captured from this adv packet
 *
 * Returns      : None
 *
 *************************************************************************/
void rssi_app_msft_update_conn_rssi_info(UINT8 handle_id, INT8 rssi)
{
    LL_CONN_HANDLE_RSSI_UNIT *pentry;

    pentry = &ll_rssi_manager.entry[handle_id];

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rssi_app_msft_update_conn_rssi_info != NULL)
    {
        if (rcp_rssi_app_msft_update_conn_rssi_info((void*)pentry, rssi))
        {
            return;
        }
    }
#endif
    
    pentry->last_rssi = rssi;

    if (pentry->start_periodic_timer)
    {
        pentry->rssi_count++;
        pentry->rssi_sum += rssi;
    } 

    if ((rssi >= pentry->max_rssi_thres) && !pentry->notified)
    {
        pentry->notified = 1;
        pentry->rssi_thres_counter = 0;

         /* Generate HCI_VS_RSSI_EVENT */
        hci_generate_vs_msft_rssi_event(HCI_COMMAND_SUCCEEDED, 
                                        pentry->conn_handle,
                                        rssi);
             
        if ((pentry->sample_period != 0x00) && 
            (pentry->sample_period != 0xFF))
        {
            pentry->start_periodic_timer = TRUE;
            pentry->sample_counter_of_tick = 0;
            pentry->rssi_count = 0;
            pentry->rssi_sum = 0;       
        }
    }

    if ((rssi > pentry->min_rssi_thres) || !pentry->notified)
    {
        pentry->rssi_thres_counter = 0;
    }
}

/**************************************************************************
 * Function     : rssi_app_msft_parser_and_store_uuid
 *
 * Description  : This function is used to parser one le advertising pdu 
 *                content and store uuid information to local catch
 *                (for faster data compare) 
 *
 * Parameters   : pdata - the pointer of payload body part of advertising pdu
 *                len - the total length of payload body
 *                uuid_set - the pointer of the catch
 *
 * Returns      : how many uuid entry stored in to catch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_parser_and_store_uuid(UINT8 *pdata, UINT8 len,
                                    LL_MSFT_HANDLE_RSSI_UUID_COMPARE_SET *uuid_set)
{
    UINT8 sign_part_len = 0;
    UINT8 ad_entry_len;
    UINT8 ad_type;
    UINT8 *ad_data;
    UINT8 counter = 0;

    if (uuid_set == NULL)
    {
        return 0;
    }

    uuid_set->uuid_16_cnt = 0;    
    uuid_set->uuid_32_cnt = 0;
    uuid_set->uuid_128_cnt = 0;

    while (sign_part_len != len)
    {
        ad_entry_len = *(pdata + sign_part_len);
        if (ad_entry_len == 0)
        {
            /* this is an early termination */
            break;
        }

        sign_part_len++;
        
        /* the definition of ad_type is n octets in the spec, 
           but it only use encode data of one byte currently */
        ad_type = *(pdata + sign_part_len);
        ad_data = pdata + sign_part_len + 1;
            
        /* increase one AD Data Structure */
        sign_part_len += ad_entry_len;

        /* avoid to cause overflow in 31 byte advertising or 
           scan response data block */
        if (sign_part_len > len)
        {
            break;
        }

        switch (ad_type)
        {
        case 0x03: /* Complete list of 16-bit UUIDs available */
        case 0x14: /* List of 16 bit Service UUIDs */   
            if (ad_entry_len == 3)
            {
                uuid_set->uuid_16[uuid_set->uuid_16_cnt] = ad_data[0] |
                                                           (ad_data[1] << 8);
                uuid_set->uuid_16_cnt++;
                counter++; 
            }
            break;
            
        case 0x05: /* Complete list of 32-bit UUIDs available */
            if (ad_entry_len == 5)
            {
                uuid_set->uuid_32[uuid_set->uuid_32_cnt] = ad_data[0] |
                                                           (ad_data[1] << 8) |
                                                           (ad_data[2] << 16) |
                                                           (ad_data[3] << 24);
                uuid_set->uuid_32_cnt++;   
                counter++; 
            }
            break;
            
        case 0x07: /* Complete list of 128-bit UUIDs available */
        case 0x15: /* List of 128 bit Service UUID */  
            if (ad_entry_len == 17)
            {
                memcpy(uuid_set->uuid_128_u1, ad_data, 16);
                uuid_set->uuid_128_cnt++;
                counter++;             
            }
            break;
            
        default:
            break;
        }
    }

    return counter;
}

/**************************************************************************
 * Function     : rssi_app_msft_compare_with_one_adv_rssi_pattern_node
 *
 * Description  : This function is used to compare all ad type entries from
 *                le advertising pdu with one node of stored pattern data
 * 
 * Parameters   : pbuf - the pointer of payload body part of advertising pdu
 *                len - the total length of payload body
 *                handle_id - assigned handle id
 *
 * Returns      : match or mismatch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_compare_with_one_adv_rssi_pattern_node(UINT8 *pbuf, UINT8 len, UINT8 handle_id)
{
    UINT8 match = FALSE;
    UINT8 sign_part_len = 0;
    UINT8 ad_entry_len;
    UINT8 ad_type;
    UINT8 *ad_data;
    LL_MSFT_HANDLE_RSSI_PATTERN_UNIT *patt;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    patt = &ll_rssi_manager.ms_adv_unit[handle_id].le_adv_pat;
#else
    patt = &ll_rssi_manager.le_adv_pat[handle_id];
#endif

    while (sign_part_len != len)
    {
        ad_entry_len = *(pbuf + sign_part_len);
        if (ad_entry_len == 0)
        {
            /* this is an early termination */
            break;
        }

        sign_part_len++;
        
        /* the definition of ad_type is n octets in the spec, 
           but it only use encode data of one byte currently */
        ad_type = *(pbuf + sign_part_len);
        ad_data = pbuf + sign_part_len + 1;
            
        /* increase one AD Data Structure */
        sign_part_len += ad_entry_len;

        /* avoid to cause overflow in 31 byte advertising or 
           scan response data block */
        if (sign_part_len > len)
        {
            break;
        }

        if (ad_type == patt->ad_type)
        {
            if ((ad_entry_len - 1) >= (patt->offset + patt->length))
            {
                if (memcmp(ad_data + patt->offset, patt->data, patt->length) == 0)
                {
                    /* pattern is match !! */
                    match = TRUE;
                    break; 
                }
            }
        }
    }

    return match;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_rssi_monitor_condition_type_uuid
 *
 * Description  : This function is used to check the condition after receive 
 *                a le advertising pdu if the condition type is uuid.
 *                It can compare all ad type are uuid (16/32/128 bits) and their
 *                data with stored uuid catch. 
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *                phandle_id - the pointer of matched handle id
 *                uuid_set - the pointer of local uuid catch from this adv packet
 *
 * Returns      : match or mismatch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_check_rssi_monitor_condition_type_uuid(UINT8 *pHdr, INT8 rssi, UINT8 *phandle_id,
                                                   LL_MSFT_HANDLE_RSSI_UUID_COMPARE_SET *uuid_set)
{
    UINT8 match = FALSE;
    UINT8 i;
    UINT8 j;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
    UINT32 bm_entry = 
        ll_rssi_manager.bm_le_adv_monitor_entry[LE_MSFT_MON_RSSI_FLAG_UUID];
#else
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_UUID_CNT;
    UINT32 bm_entry = ll_rssi_manager.bm_le_adv_monitor_entry;
#endif
    LL_MSFT_HANDLE_RSSI_UUID_UNIT *puuid;

    for (i = 0; i < imax; i++)
    { 
        if ((bm_entry == 0) || (match == TRUE))
        {
            break;
        }
        
        if (bm_entry & (1 << i))
        {
            bm_entry &= ~(1 << i);
    
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            puuid = &ll_rssi_manager.ms_adv_unit[i].le_adv_uuid;
#else
            puuid = &ll_rssi_manager.le_adv_uuid[i];
#endif

            switch (puuid->type)
            {
            case 0x01:
                for (j = 0; j < uuid_set->uuid_16_cnt; j++)
                {
                    if (uuid_set->uuid_16[j] == puuid->wdata[0])
                    {
                        /* matched UUID !! */
                        rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                        match = TRUE;
                        break;
                    }
                }
                break;
                
            case 0x02:
                for (j = 0; j < uuid_set->uuid_32_cnt; j++)
                {
                    if (uuid_set->uuid_32[j] == puuid->dwdata[0])
                    {
                        /* matched UUID !! */
                        rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                        match = TRUE;
                        break;
                    }
    
                }                    
                break;
                
            case 0x03:
                if (uuid_set->uuid_128_cnt > 0)
                {
                    if (memcmp(uuid_set->uuid_128_u1,  puuid->data, 16) == 0)
                    {
                        /* matched UUID !! */
                        rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                        match = TRUE;
                        break;
                    }
                }
                break;
                
            default:
                break;
            }
        }
    }

    *phandle_id = i;

    return match;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_rssi_monitor_condition_type_irk
 *
 * Description  : This function is used to check the condition after receive 
 *                a le advertising pdu if the condition type is irk.
 *                It can compare all stored 128 bits IRKs with one (calculate 
 *                from received le advertising pdu in random address)
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *                phandle_id - the pointer of matched handle id
 *
 * Returns      : match or mismatch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_check_rssi_monitor_condition_type_irk(UINT8 *pHdr, INT8 rssi, UINT8 *phandle_id)
{
    /*  -----------------------------------------------------------------
        randomAddress = hash || prand
        hash = ah(IRK, prand)        
        random address function ah with input parameter k set to the
        device's IRK and the input parameter r set to prand.        
        r¡¦ = padding || r        
        ah(k, r) = e(k, r') mod 2^24
    
        The securtity function e for AES-128 block cypher is supported.
        ------------------------------------------------------------------
    */

    UINT8 match = FALSE;
    UINT8 i;
    UINT16 key[8];
    UINT16 plaint_text[8] = {0, 0, 0, 0, 0, 0, 0, 0};        
    UINT16 encrypted_data[8];
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
    UINT32 bm_entry = 
        ll_rssi_manager.bm_le_adv_monitor_entry[LE_MSFT_MON_RSSI_FLAG_IRK];
#else
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_IRK_CNT;
    UINT32 bm_entry = ll_rssi_manager.bm_le_adv_monitor_entry;    
#endif 
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;    
    
    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

    /* fill r' field */
    memcpy((UINT8*)plaint_text, &pRxPkt->u1RemoteAddr[3], 3);

    for (i = 0; i < imax; i++)
    { 
        if (bm_entry == 0)
        {
            break;
        }
        
        if (bm_entry & (1 << i))
        {
            bm_entry &= ~(1 << i);

            /* fill k field */
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            memcpy((UINT8*)key, ll_rssi_manager.ms_adv_unit[i].le_adv_irk, 16);            
#else
            memcpy((UINT8*)key, &ll_rssi_manager.le_adv_irk[i][0], 16);            
#endif

            ll_driver_get_session_key(key, plaint_text, encrypted_data);

            /* check hash = ah(IRK, prand) = e(IRK, prand') mod 2^24 ? */
            if (memcmp(pRxPkt->u1RemoteAddr, (UINT8*)encrypted_data, 3) == 0)
            {
                /* matched IRK !! */
                rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                match = TRUE;
                break;
            }   
        }
    }

    *phandle_id = i;

    return match;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_rssi_monitor_condition_type_pattern
 *
 * Description  : This function is used to check the condition after receive 
 *                a le advertising pdu if the condition type is pattern.
 *                It can compare all stored patterns with one
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *                phandle_id - the pointer of matched handle id
 *
 * Returns      : match or mismatch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_check_rssi_monitor_condition_type_pattern(UINT8 *pHdr, INT8 rssi, UINT8 *phandle_id)
{
    UINT8 match = FALSE;
    UINT8 i;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
    UINT32 bm_entry = 
        ll_rssi_manager.bm_le_adv_monitor_entry[LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA];
#else
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_PATTERN_CNT;
    UINT32 bm_entry = ll_rssi_manager.bm_le_adv_monitor_entry;   
#endif  
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;   
    
    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
    
    UINT8 *pbuf = pRxPkt->AdvInd.u1AdvData;
    UINT8 len = pRxPkt->Header.Length;
    
    for (i = 0; i < imax; i++)
    {
        if (bm_entry == 0)
        {
            break;
        }
        
        if (bm_entry & (1 << i))
        {
            bm_entry &= ~(1 << i);
    
            UINT8 next = i;
            
            /* scan the linked-list */
            while (next != imax)
            {
                match = rssi_app_msft_compare_with_one_adv_rssi_pattern_node(
                                                            pbuf, len, next);
                if (match == TRUE)
                {
                    break;
                }
                
                /* switch to next node */
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
                next = ll_rssi_manager.ms_adv_unit[next].next_id;
#else
                next = ll_rssi_manager.le_adv_pat[next].next_id;
#endif
            }

            if (match == TRUE)
            {
                /* find matched data pattern !! */
                rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                break;
            }
        }
    } 

    *phandle_id = i;
    
    return match;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_rssi_monitor_condition_type_bdaddr
 *
 * Description  : This function is used to check the condition after receive 
 *                a le advertising pdu if the condition type is bd addr.
 *                It can compare all stored bt addr with one
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *                phandle_id - the pointer of matched handle id
 *
 * Returns      : match or mismatch 
 *
 *************************************************************************/
UINT8 rssi_app_msft_check_rssi_monitor_condition_type_bdaddr(UINT8 *pHdr, INT8 rssi, UINT8 *phandle_id)
{
    UINT8 match = FALSE;
    UINT8 i;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_CONDITION_CNT;
    UINT32 bm_entry = 
        ll_rssi_manager.bm_le_adv_monitor_entry[LE_MSFT_MON_RSSI_FLAG_BDADDR];
#else
    UINT8 imax = LE_MSFT_MON_RSSI_MAX_SUPPORT_BD_ADDR_CNT;
    UINT32 bm_entry = ll_rssi_manager.bm_le_adv_monitor_entry;   
#endif  
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;    
    LL_MSFT_HANDLE_RSSI_BDADDR_UNIT *pAddrStr;
    
    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

    for (i = 0; i < imax; i++)
    {
        if (bm_entry == 0)
        {
            break;
        }
        
        if (bm_entry & (1 << i))
        {
            bm_entry &= ~(1 << i);
    
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            pAddrStr = &ll_rssi_manager.ms_adv_unit[i].le_adv_bd_addr;
#else
            pAddrStr = &ll_rssi_manager.le_adv_bd_addr[i];
#endif

    
            if ((pRxPkt->Header.TxAdd == pAddrStr->type) &&
                (memcmp(pRxPkt->u1RemoteAddr, pAddrStr->addr, 6) == 0))
            {
                /* matched bd_addr */
                rssi_app_msft_update_adv_rssi_info(i, pHdr, rssi);
                match = TRUE;
                break;
            }
        }
    }

    *phandle_id = i;

    return match;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_rssi_monitor_condition
 *
 * Description  : This function is used to check the condition after receive 
 *                a le advertising pdu for different condition type.
 *                Then it can decide we shall propagate the advertisement to the 
 *                host or not 
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *
 * Returns      : propagate the advertisement to the host or not 
 *
 *************************************************************************/
UINT8 rssi_app_msft_check_rssi_monitor_condition(UINT8 *pHdr, INT8 rssi)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 i = 0;
    UINT8 match = FALSE;
    UINT8 send_adv = TRUE;
    UINT8 counter;
    UINT8 mon_type;
    UCHAR status;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    UINT8 check_white_list = TRUE;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_MATCH_MULTI_CONDI_
    UINT8 send_adv_multi_condi_bm = 0;    
#endif
#endif

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rssi_app_msft_check_rssi_monitor_condition != NULL)
    {
        if (rcp_rssi_app_msft_check_rssi_monitor_condition(pHdr, rssi, &status))
        {
            return status;
        }
    }
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_    
    /* check multiple condition */
    for (mon_type = LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA; 
            mon_type <= LE_MSFT_MON_RSSI_FLAG_BDADDR; 
                mon_type++)
#endif
    {
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        match = FALSE;

        if (!(ll_rssi_manager.le_adv_monitor_type & (1 << mon_type)))
        {
            continue;
        }
#else
        mon_type = ll_rssi_manager.le_adv_monitor_type;
#endif

        switch (mon_type)
        {
        case LE_MSFT_MON_RSSI_FLAG_UUID:
            if (pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
            {
                /* directed advertising pdu has no data field */
                break;
            }

            /* follow avertising or scan response data format of GAP (vol 3). */
            
            /* worse case is 16 bits uuid of seven sets in the adv pdu */
            LL_MSFT_HANDLE_RSSI_UUID_COMPARE_SET uuid_set;

            counter = rssi_app_msft_parser_and_store_uuid(pRxPkt->AdvInd.u1AdvData,
                                                          pRxPkt->Header.Length,
                                                          &uuid_set);
            if (counter == 0)
            {
                /* no any UUID content */
                break;
            }
            match = rssi_app_msft_check_rssi_monitor_condition_type_uuid(pHdr, rssi, &i, &uuid_set);
            break;
                
        case LE_MSFT_MON_RSSI_FLAG_IRK:
            if (pRxPkt->Header.TxAdd == 0)
            {
                /* quit !! because it is a public bd address */
                break;
            }
            match = rssi_app_msft_check_rssi_monitor_condition_type_irk(pHdr, rssi, &i);
            break;

        case LE_MSFT_MON_RSSI_FLAG_PATTERN_DATA:
            if (pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
            {
                /* directed advertising pdu has no data field */
                break;
            }
            match = rssi_app_msft_check_rssi_monitor_condition_type_pattern(pHdr, rssi, &i);
            break;

        case LE_MSFT_MON_RSSI_FLAG_BDADDR:
            match = rssi_app_msft_check_rssi_monitor_condition_type_bdaddr(pHdr, rssi, &i);    
            break;
            
        default:
            break;
        }    

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_MATCH_MULTI_CONDI_
        if (match)
        {
            /* find any matched patterns, so we can quit now !! */
            break;
        }
#else
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        /* If Enable is set to 0x00, the controller SHALL propagate the 
           advertisement to the host based on existing whitelist settings. */
        if (match && ll_rssi_manager.le_adv_monttor_filter_enable)
#else
        if (match)
#endif
        {
            send_adv = FALSE;

            if (ll_rssi_manager.adv_entry[i].mon_state == LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE)
            {
                /* If the RSSI_Sampling_Period is set to 0x0 then the controller 
                   SHALL propagate all received advertisement packets to the host 
                   for this device (for this Condition). */
                if (ll_rssi_manager.adv_entry[i].sample_period == 0)
                {
                    send_adv = TRUE;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_MATCH_MULTI_CONDI_
                    send_adv_multi_condi_bm |= 1 << mon_type;    
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ 
                    check_white_list = FALSE;        
#endif
                }
            }

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
            /* If Enable is set to 0x00, the controller SHALL propagate the 
               advertisement to the host based on existing whitelist settings. */
            if (ll_rssi_manager.le_adv_monttor_filter_enable == 0)
            {
                send_adv = TRUE;
            }
#endif        
        }
#endif
#endif        
    }

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_MATCH_MULTI_CONDI_
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
    /* If Enable is set to 0x00, the controller SHALL propagate the 
       advertisement to the host based on existing whitelist settings. */
    if (match && ll_rssi_manager.le_adv_monttor_filter_enable)
#else
    if (match)
#endif
    {
        send_adv = FALSE;

        if (ll_rssi_manager.adv_entry[i].mon_state == LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE)
        {
            /* If the RSSI_Sampling_Period is set to 0x0 then the controller 
               SHALL propagate all received advertisement packets to the host 
               for this device (for this Condition). */
            if (ll_rssi_manager.adv_entry[i].sample_period == 0)
            {
                send_adv = TRUE;

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_ 
                check_white_list = FALSE;        
#endif
            }
        }

#ifndef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
        /* If Enable is set to 0x00, the controller SHALL propagate the 
           advertisement to the host based on existing whitelist settings. */
        if (ll_rssi_manager.le_adv_monttor_filter_enable == 0)
        {
            send_adv = TRUE;
        }
#endif        
    }
#endif

#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_MATCH_MULTI_CONDI_
    if (send_adv_multi_condi_bm > 0)                   
    {
        check_white_list = FALSE;
    }
#endif

    if (check_white_list)
    {        
        if (ll_manager.scan_unit.sw_filter_policy)
        {
            if (ll_driver_search_dev_from_list(LL_WHITE_LIST_TYPE, 
                      pRxPkt->Header.TxAdd, pRxPkt->u1RemoteAddr) !=
                      LL_MAX_WHITE_LIST_SIZE)
            {
                send_adv = TRUE;

                if (pRxPkt->Header.PDU_Type == LL_ADV_PDU_TYPE_ADV_DIRECT_IND)
                {
                    /* check directed adv pdu */
                    
                    UINT8 *pMyAddr;
                    
                    /* check RxAddr is my address ? */
                    if (pRxPkt->Header.RxAdd)
                    {
                        pMyAddr = ll_manager.local_random_address;
                    }
                    else
                    {
                        pMyAddr = ll_manager.local_public_address;          
                    }
    
                    if (memcmp(pMyAddr, pRxPkt->AdvDirInd.InitA.u1Addr, 6) != 0)
                    {
                        send_adv = FALSE;           
                    }
                }
            }
            else
            {
                send_adv = FALSE;
            }
        }
        else
        {
            send_adv = TRUE;
        }
    }
#endif

    return send_adv;
}

/**************************************************************************
 * Function     : rssi_app_msft_check_conn_thres_counter_then_send_event
 *
 * Description  : This function is used to check rssi threshold timer and 
 *                periodical sample timer for monitored BR/EDR or BLE 
 *                connection. It will send a hci_generate_vs_msft_rssi_event 
 *                to the host if any timer is expired.
 * 
 * Parameters   : handle_id - assigned handle id
 *
 * Returns      : None 
 *
 *************************************************************************/
void rssi_app_msft_check_conn_thres_counter_then_send_event(UINT8 handle_id)
{
    LL_CONN_HANDLE_RSSI_UNIT *pentry;
    INT8 adv_rssi;

    pentry = &ll_rssi_manager.entry[handle_id];

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rssi_app_msft_check_conn_thres_counter_then_send_event != NULL)
    {
        if (rcp_rssi_app_msft_check_conn_thres_counter_then_send_event(pentry, handle_id))
        {
            return;
        }
    }
#endif

    if (pentry->notified)
    {
        pentry->rssi_thres_counter++;
        if (pentry->rssi_thres_counter >= pentry->min_rssi_thres_intv)
        {
            pentry->rssi_thres_counter = 0;
            pentry->start_periodic_timer = FALSE;
            pentry->notified = 0;

            /* Generate HCI_VS_RSSI_EVENT */
            hci_generate_vs_msft_rssi_event(HCI_COMMAND_SUCCEEDED, 
                                            pentry->conn_handle,
                                            pentry->last_rssi);
        }
    }

    if (pentry->start_periodic_timer)
    {
        pentry->sample_counter_of_tick++;
        if (pentry->sample_counter_of_tick >= pentry->sample_period)
        {
            pentry->sample_counter_of_tick = 0;
            
            /* Propagate advertisement to host with AVERAGE RSSI 
               received during this sampling period */
    
            if (pentry->rssi_count > 0)
            {
                adv_rssi = pentry->rssi_sum / pentry->rssi_count;
                hci_generate_vs_msft_rssi_event(HCI_COMMAND_SUCCEEDED, 
                                                pentry->conn_handle,
                                                adv_rssi);
                pentry->rssi_count = 0;
                pentry->rssi_sum = 0;
            } 
        }
    }
}

/**************************************************************************
 * Function     : rssi_app_msft_ll_scanner_generate_le_adv_report
 *
 * Description  : This function is used to generate a le advertising report. 
 *                periodical sample timer for monitored le advertising pdu. 
 *                It will send a hci_generate_vs_msft_le_monitor_event 
 *                to the host if any timer is expired.
 * 
 * Parameters   : pHdr - the pointer of the head of advertising pdu
 *                rssi - the rssi value captured from this adv packet
 *
 * Returns      : None 
 *
 *************************************************************************/
void rssi_app_msft_ll_scanner_generate_le_adv_report(UINT8 *pHdr, INT8 rssi)
{
    LE_HW_ADVERTISING_CH_RX_PKT_S *pRxPkt;
    UINT8 pdu_type;
    UINT8 data_buf_size;
    LL_ADV_REPORT_UNIT *rpt;
    UINT8 index;

    pRxPkt = (LE_HW_ADVERTISING_CH_RX_PKT_S *)pHdr;
    pdu_type = pRxPkt->Header.PDU_Type;

    switch (pdu_type)
    {
        case LL_ADV_PDU_TYPE_ADV_DIRECT_IND:
            data_buf_size = 0;
            break;

        case LL_ADV_PDU_TYPE_ADV_IND:
        case LL_ADV_PDU_TYPE_ADV_DISCOVER_IND:
        case LL_ADV_PDU_TYPE_ADV_NONCONN_IND:
        case LL_ADV_PDU_TYPE_SCAN_RSP:
            data_buf_size = pRxPkt->Header.Length - 6;
            break;

        default:
            return;
    }

    /* fw can queue the advertising report in one LE Advertising Report event */
    rpt = (LL_ADV_REPORT_UNIT *)&ll_manager.scan_unit.report;
    index = rpt->num_of_reports;
    rpt->event_type[index] = pRxPkt->Header.PDU_Type;
    rpt->addr_type[index] = pRxPkt->Header.TxAdd;
    memcpy(rpt->addr[index], pRxPkt->AdvInd.AdvA.u1Addr, 6);
    rpt->data_len[index] = data_buf_size;
    if (data_buf_size > 0)
    {
        memcpy(rpt->data_buf[index], pRxPkt->AdvInd.u1AdvData, data_buf_size);
    }

    rpt->rssi[index] = *(UINT8*)&rssi;
    rpt->num_of_reports++;

    if (rpt->num_of_reports > 0)
    {
        hci_generate_le_advertising_report_event();
        rpt->num_of_reports = 0;
    }
}

/**************************************************************************
 * Function     : rssi_app_msft_check_adv_counter_then_send_event
 *
 * Description  : This function is used to check rssi threshold timer and 
 *                periodical sample timer for monitored le advertising pdu. 
 *                It will send a hci_generate_vs_msft_le_monitor_event 
 *                to the host if any timer is expired.
 * 
 * Parameters   : handle_id - assigned handle id
 *
 * Returns      : None 
 *
 *************************************************************************/
void rssi_app_msft_check_adv_counter_then_send_event(UINT8 handle_id)
{
    LL_ADV_RSSI_UNIT *pentry;
    LE_HW_ADVERTISING_CH_RX_PKT_S *padv;
    INT8 adv_rssi;

    pentry = &ll_rssi_manager.adv_entry[handle_id];

#ifdef _ROM_CODE_PATCHED_
    if (rcp_rssi_app_msft_check_adv_counter_then_send_event != NULL)
    {
        if (rcp_rssi_app_msft_check_adv_counter_then_send_event(pentry, handle_id))
        {
            return;
        }
    }
#endif

    if (pentry->mon_state == LE_MSFT_MONITOR_RSSI_STATE_IN_RANGE)
    { 
        pentry->rssi_thres_counter++;
        if (pentry->rssi_thres_counter >= pentry->min_rssi_thres_intv)
        {
            pentry->rssi_thres_counter = 0;
            pentry->mon_state = LE_MSFT_MONITOR_RSSI_STATE_OUT_RANGE;
            pentry->notified = 0;
            pentry->start_periodic_timer = FALSE;
            
            /* TODO: The controller SHALL propagate an advertisement packet to 
               the host when the received RSSI is less than or equal to 
               RSSI_Threshold_Low over the specified 
               RSSI_Threshold_Low_TimeInterval for a particular device (for 
               this Condition). The RSSI value of this advertisement packet 
               SHALL be the RSSI value of the last advertisement received. */
            adv_rssi = pentry->last_rssi;
            rssi_app_msft_ll_scanner_generate_le_adv_report(
                        (UINT8*)&pentry->loc_adv,  adv_rssi);


            /* The controller SHALL generate an 
               HCI_VS_MSFT_LE_Monitor_Device_Event event with MonitorState set 
               to 0, to notify the host that the controller has stopped 
               monitoring that particular device (for this Condition) */
            padv = &(pentry->loc_adv);
            hci_generate_vs_msft_le_monitor_event(padv->Header.TxAdd,
                                   padv->u1RemoteAddr, handle_id, 0);
            
        }

        if (pentry->start_periodic_timer)
        {
            pentry->sample_counter_of_tick++;
            if (pentry->sample_counter_of_tick >= pentry->sample_period)
            {
                pentry->sample_counter_of_tick = 0;
                
                /* Propagate advertisement to host with AVERAGE RSSI 
                   received during this sampling period */

                if (pentry->pkt_cnts > 0)
                {
                    adv_rssi = pentry->pkt_rssi_sum / pentry->pkt_cnts;
                    rssi_app_msft_ll_scanner_generate_le_adv_report(
                                (UINT8*)&pentry->loc_adv,  adv_rssi);

                    pentry->pkt_cnts = 0;
                    pentry->pkt_rssi_sum = 0;
                } 
            }
        }
    }
}

void rssi_app_msft_check_all_counter_every_100ms(void)
{
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_
    if (ll_rssi_manager.msft_monitor_rssi_mode == 
                    LE_MSFT_MONITOR_RSSI_VIA_CONN_HANDLE_MODE_MSFT)
    {
        g_10ms_timer_tick_cnt++;
        if (g_10ms_timer_tick_cnt == 10)
        {
            UINT8 i;
            UINT32 tmp32;
            UINT16 tmp16;
            UINT8 tmp8;
            UINT16 ce_index;
                    
            /* monitor BR/EDR connection */
            i = 0;
            tmp16 = ll_rssi_manager.bm_used_legacy_handle;
            while (tmp16)
            {
                if (tmp16 & 0x0001)
                {
                    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(ll_rssi_manager.entry[i].conn_handle, 
                                                &ce_index) == API_SUCCESS)
                    {
                        rssi_app_msft_check_conn_thres_counter_then_send_event(i);              
                    }
                }
                tmp16 >>= 1;
                i++;
            }
        
            /* monitor BLE connection */
            i = 0;
            tmp8 = ll_rssi_manager.bm_used_handle;
            while (tmp8)
            {
                if (tmp8 & 0x01)
                {
                    if (ll_manager.conn_unit.enable &&
                        (ll_manager.conn_unit.bmActiveHandle & (1 << i)))
                    rssi_app_msft_check_conn_thres_counter_then_send_event(i + 16);
                }
                tmp8 >>= 1;
                i++;           
            }
        
            /* monitor BLE advertising */
            if (ll_manager.scan_unit.enable)
            {
                i = 0;
#ifdef _SUPPORT_MSFT_BT_HCI_EXTENSION_FINAL_SPEC_
                tmp32 = ll_rssi_manager.bm_le_adv_monitor_entry[0];
#else
                tmp32 = ll_rssi_manager.bm_le_adv_monitor_entry;
#endif
                while (tmp32)
                {
                    if (tmp32 & 0x00000001)
                    {
                        rssi_app_msft_check_adv_counter_then_send_event(i);                
                    }
                    tmp32 >>= 1;
                    i++;            
                }
            }
            g_10ms_timer_tick_cnt = 0;
        }
    }
#endif
}

#endif

#ifdef _SUPPORT_AUTO_DETACH_LINK_
TimerHandle_t auto_detach_link_timer = NULL;
UINT8 auto_detach_link_current_mode = 0xff;
OS_SIGNAL catch_os_signal;

UINT8 auto_detach_check_all_link_removed(void)
{
    UINT8 i;

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* terminate possible le acl links */
        for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
        {
            if (ll_manager.conn_unit.handle[i].connected &&
                (ll_manager.conn_unit.bmActiveHandle & (1 << i)))
            {
                return FALSE;
            }
        }
    }
#endif

    /* terminate possible esco links */    
    for(i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i++)
    {
        if ((lmp_esco_connection_entity[i].entity_status == ASSIGNED) &&
            (lmp_esco_connection_entity[i].status == ESCO_CONNECTED))
        {
            return FALSE;
        }
    }

    /* terminate possible sco links */        
    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        if ((lmp_sco_connection_data[i].sco_conn_status != SCO_FREE) &&
            (lmp_sco_connection_data[i].sco_conn_status == SCO_CONNECTED))
        {
            return FALSE;
        }
    }

    /* terminate possible acl links */    
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        if (lmp_connection_entity[i].entity_status == ASSIGNED)
        {
            return FALSE;
        }
    }

    return TRUE;
}

UINT8 auto_detach_terminate_all_remote_links(void)
{
    UINT8 i;
    HCI_CMD_PKT temp_hci_cmd_pkt;
    UINT8 temp_sent_event_flag;
    UINT16 conn_handle;
    UINT32 filter = 0;
    UINT8 count = 0;
    UINT8 result;
        
    temp_hci_cmd_pkt.cmd_opcode = HCI_DISCONNECT_OPCODE;
    temp_hci_cmd_pkt.param_total_length = 3;    
    temp_hci_cmd_pkt.cmd_parameter[2] = CONNECTION_TERMINATED_USER_ERROR;

#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* terminate possible le acl links */
        for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
        {
            if (ll_manager.conn_unit.handle[i].connected &&
                (ll_manager.conn_unit.bmActiveHandle & (1 << i)))
            {
                conn_handle = ll_manager.conn_unit.handle[i].conn_handle;
                temp_hci_cmd_pkt.cmd_parameter[0] = LSB(conn_handle);
                temp_hci_cmd_pkt.cmd_parameter[1] = MSB(conn_handle);
                result = hci_handle_disconnect_command(&temp_hci_cmd_pkt,
                                                &temp_sent_event_flag);            
                count++;
            }
        }
    }
#endif

    /* terminate possible esco links */    
    for (i = 0; i < LMP_MAX_ESCO_CONN_ENTITIES; i++)
    {
        if ((lmp_esco_connection_entity[i].entity_status == ASSIGNED) &&
            (lmp_esco_connection_entity[i].status == ESCO_CONNECTED))
        {
            conn_handle = lmp_esco_connection_entity[i].conn_handle;
            filter |= 1 << lmp_esco_connection_entity[i].ce_index;
            temp_hci_cmd_pkt.cmd_parameter[0] = LSB(conn_handle);
            temp_hci_cmd_pkt.cmd_parameter[1] = MSB(conn_handle);
            result = hci_handle_disconnect_command(&temp_hci_cmd_pkt,
                                            &temp_sent_event_flag);     
            if (result == HCI_COMMAND_SUCCEEDED)
            {
                count++;
            }
        }
    }

    /* terminate possible sco links */        
    for (i = 0; i < LMP_MAX_SCO_CONNECTIONS; i++)
    {
        if ((lmp_sco_connection_data[i].sco_conn_status != SCO_FREE) &&
            (lmp_sco_connection_data[i].sco_conn_status == SCO_CONNECTED))
        {
            conn_handle = lmp_sco_connection_data[i].sco_conn_handle;
            filter |= 1 << lmp_sco_connection_data[i].conn_entity_index;
            temp_hci_cmd_pkt.cmd_parameter[0] = LSB(conn_handle);
            temp_hci_cmd_pkt.cmd_parameter[1] = MSB(conn_handle);
            result = hci_handle_disconnect_command(&temp_hci_cmd_pkt,
                                            &temp_sent_event_flag);             
            if (result == HCI_COMMAND_SUCCEEDED)
            {
                count++;
            }
        }
    }

    /* terminate possible acl links */    
    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        if (lmp_connection_entity[i].entity_status == ASSIGNED)
        {
            if (filter & i)
            {
                /* this entity is handled when disconnect sco or esco link */
                continue;
            }
            
            conn_handle = lmp_connection_entity[i].connection_type.connection_handle;
            temp_hci_cmd_pkt.cmd_parameter[0] = LSB(conn_handle);
            temp_hci_cmd_pkt.cmd_parameter[1] = MSB(conn_handle);
            result = hci_handle_disconnect_command(&temp_hci_cmd_pkt,
                                            &temp_sent_event_flag);  

            if (result == HCI_COMMAND_SUCCEEDED)
            {
                count++;
            }
        }
    }

#if 0
    if (count > 0)
    {
        RT_BT_LOG(RED, DAPE_TEST_LOG293, 1, count);
    }
#endif

    return count;
}

void auto_detach_terminate_all_remote_links_task(void *no_arg, uint32_t no_arg2)
{
    auto_detach_terminate_all_remote_links();
}

UINT8 auto_detach_pend_terminate_all_remote_links_from_isr(void)
{
    UINT8 count = 0;
    UINT8 i;
#ifdef LE_MODE_EN
    if (IS_BT40)
    {
        /* terminate possible le acl links */
        for (i = 0; i < LL_MAX_CONNECTION_UNITS; i++)
        {
            if (ll_manager.conn_unit.handle[i].connected &&
                (ll_manager.conn_unit.bmActiveHandle & (1 << i)))
            {
                ++count;
            }
        }
    }
#endif
    count += lmp_self_device_data.number_of_acl_conn
            + lmp_self_device_data.total_no_of_sco_conn
            + lmp_self_device_data.number_of_esco_connections;
    if (count > 0)
    {
        BaseType_t high_pri_task_woken = pdFAIL;
        xTimerPendFunctionCallFromISR(
                auto_detach_terminate_all_remote_links_task, NULL, 0,
                &high_pri_task_woken);
        portYIELD_FROM_ISR(high_pri_task_woken);
    }
    return count;
}

void auto_detach_link_callback(TimerHandle_t timer_handle)
{
    UINT8 mode = (UINT8)((UINT32) pvTimerGetTimerID(timer_handle));

    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();
    if (auto_detach_link_timer != NULL)
    {
        OS_DELETE_TIMER(&auto_detach_link_timer);
        auto_detach_link_current_mode = 0xff;

        switch (mode)
        {
        case AUTO_DETACH_QUEUE_HCI_RESET_CMD:
            if (auto_detach_terminate_all_remote_links() > 0)
            {                 
                auto_detach_enable_link_timer(
                            AUTO_DETACH_PROCESS_HCI_RESET_CMD, 2000);
                break;
            }
            /* pass through to case 0 */
            
        case AUTO_DETACH_PROCESS_HCI_RESET_CMD:
            {
                HCI_CMD_PKT *hci_cmd_pkt_ptr ;
                hci_cmd_pkt_ptr = (HCI_CMD_PKT *)catch_os_signal.param ;
                if ((hci_cmd_pkt_ptr->cmd_opcode) == HCI_RESET_OPCODE)
                {
                    HCI_Command_Task(&catch_os_signal);
                    memset(&catch_os_signal, 0, sizeof(OS_SIGNAL));
                }
            }
            break;

        case AUTO_DETACH_PROCESS_HW_RADIO_OFF:
            {
                do 
                {
                    /* check GPIO 11 level again */                
                    if (GPIO_READ(GPIO_EXT_PORTA_REG) & 0x01)
                    {
                        break;
                    }
                                    
                    BTON_POW_CTRL_REG_S_TYPE pow_ctrl;
                    pow_ctrl.d32 = VENDOR_READ(BTON_POW_CTRL_REG);
                    pow_ctrl.b.bt_hwpdn_en = 1;
                    VENDOR_WRITE(BTON_POW_CTRL_REG, pow_ctrl.d32);    
                }
                while (0);              
            }
            break;

#ifdef CONFIG_TV_POWERON
        case AUTO_DETACH_TV_OFF_CMD:
            {
                WDG_TIMER_TIMEOUT_SOON;
                while (1) {} /* infinite loop here to wait watch dog timeout */
            }
            break;
#endif

        default:
            break;
        }
    }
    MINT_OS_EXIT_CRITICAL();
}

void auto_detach_enable_link_timer(UINT8 mode, UINT16 time_ms)
{    
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

//    RT_BT_LOG(RED, DAPE_TEST_LOG525, 6, 
//                    auto_detach_link_current_mode, mode, time_ms,0,0,0);

    if ((auto_detach_link_current_mode == AUTO_DETACH_PROCESS_HW_RADIO_OFF) && 
        (mode != AUTO_DETACH_PROCESS_HW_RADIO_OFF))
    {
        MINT_OS_EXIT_CRITICAL();        

        /* TBD: let hw radio off's priority is higher than hci reset command */
        return;
    }
        
    if (auto_detach_link_timer != NULL)
    {        
        OS_DELETE_TIMER(&auto_detach_link_timer);
    }

    /* Create a tid timer */
    OS_CREATE_TIMER(ONESHOT_TIMER, &auto_detach_link_timer,
            auto_detach_link_callback, (void *)((UINT32)mode), 0);

    auto_detach_link_current_mode = mode;

    OS_START_TIMER(auto_detach_link_timer, time_ms);

    MINT_OS_EXIT_CRITICAL();
}
#endif

