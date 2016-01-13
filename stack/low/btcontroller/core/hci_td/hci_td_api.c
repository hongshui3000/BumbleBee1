/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI Transport Driver API implementation.
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 32 };
/********************************* Logger *************************/

#include "hci_td_api.h"
#include "hci_td.h"
#include "bt_fw_os.h"
#include "hci_vendor_defines.h"
#include "lmp_1_2_defines.h"
#include "lmp.h"
#include "lc.h"
#include "platform.h"
#include "bt_fw_hci.h"
#include "mem.h"

#include "bt_fw_acl_q.h"

#ifdef PTA_EXTENSION
#include "pta_meter.h"
#endif
void hci_td_tx_packet_data(UCHAR pkt_type, UCHAR *buffer, UINT16 length, 
                                 UCHAR *data_buffer, UINT16 data_length);


PF_HCI_TRANSPORT_WRITE_FN pf_hci_transport_write;
PF_TX_FIFO_ORDER_FREE_FN pf_tx_fifo_order_free;
PF_SWITCH_HCI_DMA_PARAMTER_FN pf_switch_hci_dma_parameter;
PF_INTERFACE_WAKE_UP_INTERRUPT_FN pf_interface_wake_up_interrupt;
PF_ENABLE_USB_INTERFACE_SCO_FILTER_FN pf_enable_usb_interface_sco_filter;

OS_HANDLE hci_td_task_handle;

HCI_TD_RX_TABLE  rx_table[HCI_TD_MAX_HANDLER_TASKS]; /* from host */
HCI_TD_TX_TABLE  tx_table[HCI_TD_MAX_HANDLER_TASKS]; /* to host */

#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)
extern POOL_ID synchronous_data_to_host_pool_id;
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */


#ifdef COMPILE_ESCO
extern API_RESULT lmp_get_esco_ce_index_from_conn_handle(UINT16 conn_handle,
                                                UINT16* ce_index);
#endif


/** 
 * Initializes the HCI Transport Driver.
 * This API will install the callback functions with the UART
 * Initialize the HCI TD's internal data structures, create the
 * HCI TD Task and Create the HCI TD's packet receive Timer
 * 
 * \return SUCCESS always.
 */
API_RESULT hci_td_init(void)
{
	//TD_INF(log_file,"Initializing Transport Driver \n");
	TD_INF(INITIALIZING_TRANSPORT_DRIVER,0,0);

    RxPktFunc = hci_td_handle_received_packet;
    TxCompFunc = hci_td_handle_tx_completed;
	pf_hci_transport_write = S3C2410Usb_Dma; 
	pf_tx_fifo_order_free = dma_tx_fifo_order_free;
	pf_switch_hci_dma_parameter = switch_hci_dma_setting;
    pf_interface_wake_up_interrupt = wake_up_interrupt_switch;
    pf_enable_usb_interface_sco_filter = enable_filter_sco_connection;

    pf_hci_transport_init();

#ifdef _8821A_BTON_DESIGN_
    /* fast_gpio_power_on_check option is to be tested */
    EFUSE_POW_SETTING_4_S efuse_pow_setting_4;
    *(UINT8*)&efuse_pow_setting_4 = otp_str_data.efuse_pow_setting_4_d8;
    if (efuse_pow_setting_4.fast_gpio_power_on_check2b==1)
    {   
        GpioInit();
        power_init();
    }
#endif

    //TD_INF(log_file,"Transport Driver Initialized Successfully");
	TD_INF(TRANSPORT_DRIVER_INITIALIZED_SUCCESSFULLY,0,0);
    return API_SUCCESS;
}

/** 
 * API provided as a call back function for the UART and USB Interrupt
 * handlers. This function will be invoked whenever tranmission of the given
 * packet is completed by the platform transport driver.
 * 
 * \param buffer Pointer to the transmitted packet.
 * \param len Length of the packet that was given for transmission.
 * 
 * \return None.
 */
void hci_td_handle_tx_completed(UCHAR *buffer, UINT16 len, UINT8 type)
{
#ifdef _FAST_HCI_TD_COMPLETE_
    hci_td_tx_complete(buffer, type);
#else
	OS_SIGNAL signal ;
	signal.type  = HCI_TD_HANDLE_TX_COMPLETED_SIGNAL;
	signal.param = (OS_ADDRESS)buffer;
	signal.length = 1;
	signal.fifo_type = type;
	OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, signal);
#endif
}

/** 
 * API provided to deliver the Event packets to the HCI TD Task.
 * This API sends an event to the HCI TD task, which
 * will actually complete writing the Event packet to the UART/USB.
 * Will be used by Bluetooth firmware stack modules when an
 * Event packet has to be sent to the host.
 * 
 * \param event_ptr Pointer to the HCI Event Packet without the 
 *                  packet type indicator.
 * 
 * \return SUCCESS always.
 */
API_RESULT hci_td_deliver_event_to_host( UCHAR * event_ptr)
{
    UINT16 param_len;
    OS_SIGNAL signal ;

    signal.type  = HCI_TD_HCI_EVENT_TO_HOST_SIGNAL;
    signal.param = (OS_ADDRESS *)event_ptr;

    /* Get the length of the Event parameters */
    param_len = event_ptr[1];

   /* Total length of the event packet = 
    * param_length (2nd  byte)+ Event type (1) + length field(1) 
    */
    signal.length = (UINT16)(param_len + 1 + 1);

    OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle, signal);

	TD_INF(EVENT_SENT_TO_HCI_TD_TASK_EVENT_TYPE_PARAM_LENGTH,2,*event_ptr,param_len);
    return API_SUCCESS;
}

/** 
 * API provided to deliver the ACL data to the HCI TD Task.
 * This API sends an event to the HCI TD task, which
 * will actually complete writing the packet to the UART/USB.
 * Will be used by Bluetooth firmware stack modules when an
 * ACL data packet has to be sent to the host.
 * 
 * \param header ACL Data packet header.
 * \param pkt_ptr The packet pointer (payload).
 * 
 * \return SUCCESS always.
 */
API_RESULT hci_td_deliver_acl_data_to_host(UCHAR *header, UCHAR *pkt_ptr )
{
    UINT16 length;

    length = (UINT16)((header[3] << 8) | header[2]);

	TD_INF(SIGNAL_PARAM,1, header);

	// 20111026 morgan add ACL RX meter
	//20120302 : morgan mark , count RX 2 times , shall reduce 1 time.
/*	
#ifdef PTA_EXTENSION
    if (is_wifi_alive)
    {
    	if( pta_meter_var.bPtaMeterSwitch )
    	{
    		pta_meter_var.dwPtaACLRxCnt += length;
    	}
    }
#endif	
*/
    hci_td_tx_packet_data(HCI_TRANSPORT_ACL_DATA_PKT_TYPE, header, 4,
                          pkt_ptr, length);

    return API_SUCCESS;
}


#if defined(SCO_OVER_HCI) || defined(COMPILE_ESCO)

/** 
 * API provided to deliver the SYNC data to the 
 * HCI TD Task. This API will send a signal  to the hci_td task
 * which will write the packet over the Transport interface.
 * 
 * \param pkt_ptr Pointer to the HCI SYNC Packet.
 * 
 * \return API_SUCCESS or API_FAILURE.
 */


API_RESULT hci_td_deliver_synchronous_data_to_host(UCHAR *pkt_ptr)
{
    UCHAR sync_data_length;
    OS_SIGNAL signal;    
    UINT16 conn_handle;

#ifdef COMPILE_ESCO
    UINT16 esco_ce_index = INVALID_CE_INDEX;
#endif

#if defined SCO_OVER_HCI || defined COMPILE_ESCO
    UINT16 sco_ce_index = INVALID_CE_INDEX;
#endif

#ifdef TEST_MODE
#ifdef COMPILE_ESCO
    UCHAR *test_mode_esco_pkt_buf = NULL;
    UINT16 ce_index;
#endif     
#endif   
   
	conn_handle = pkt_ptr[2];
    conn_handle = (UINT16)(conn_handle << 8);
    conn_handle = (UINT16)(conn_handle | pkt_ptr[1]);
    
    /* Get the Length of the Synchronous data */
    sync_data_length = pkt_ptr[3];  
    
    /* This if should be removed once sco-connection completion event is 
       generated from write_loop_back command */

	// this code will never be execute
#ifdef TEST_MODE       
    if( lmp_self_device_data.test_mode != HCI_LOCAL_LOOPBACK_MODE)
#endif /* TEST_MODE */    
    {

#if defined(COMPILE_ESCO) || defined(SCO_OVER_HCI)
    
#if defined(COMPILE_ESCO) && !defined(SCO_OVER_HCI)
        if(lmp_get_esco_ce_index_from_conn_handle(conn_handle,&esco_ce_index) 
                != API_SUCCESS)
#elif !defined(COMPILE_ESCO) && defined(SCO_OVER_HCI)
        if (lmp_get_sco_ce_index_from_conn_handle(conn_handle, &sco_ce_index) 
                != API_SUCCESS)
#elif defined(COMPILE_ESCO) && defined(SCO_OVER_HCI)
        if (lmp_get_esco_ce_index_from_conn_handle(conn_handle, &esco_ce_index)
                != API_SUCCESS &&
            lmp_get_sco_ce_index_from_conn_handle(conn_handle, &sco_ce_index)
                != API_SUCCESS)
#endif /* defined(COMPILE_ESCO) && !defined(SCO_OVER_HCI) */
        {
            return API_FAILURE;
        }      
#endif /* defined(COMPILE_ESCO) || defined(SCO_OVER_HCI) */
    
#ifdef TEST_MODE
#ifdef COMPILE_ESCO
        if( lmp_self_device_data.test_mode == HCI_DUT_LOOPBACK_MODE ||
            lmp_self_device_data.test_mode == HCI_DEVICE_UNDER_TEST_MODE)
        {
            ce_index = lmp_esco_connection_entity[esco_ce_index].ce_index;

            if(lmp_connection_entity[ce_index].test_mode_info.test_state == 
                                                                    TEST_STARTED )
            {
//chris_text_mode_modify
                if(OS_ALLOC_BUFFER(rx_table[SYNCHRONOUS_DATA_HANDLER_TASK].pool_handle,
                                  (void **) &test_mode_esco_pkt_buf) != BT_ERROR_OK)
                {
                    //ESCO_ERR(log_file, "Esco buffer alloc. failed\n");
					ESCO_ERR(ESCO_BUFFER_ALLOC_FAILED,0,0);
                    return API_SUCCESS;
                }

                memcpy(test_mode_esco_pkt_buf,pkt_ptr,sync_data_length + 3);

                /* Freeing the esco receive buffer */
                OS_FREE_BUFFER(synchronous_data_to_host_pool_id,pkt_ptr);

                signal.type = LC_HANDLE_HOST_SYNCHRONOUS_PKT;
                signal.length =  (UINT16)(sync_data_length + 3);
                signal.param = test_mode_esco_pkt_buf;            
                /* Send the Signal to LC Task to process the Data Pkt */
                OS_SEND_SIGNAL_TO_TASK(rx_table[SYNCHRONOUS_DATA_HANDLER_TASK].
                                       task_handle,signal);            
                return API_SUCCESS;
            } 
        }    
#endif
#endif
    }

    signal.type  = HCI_TD_SYNCHRONOUS_DATA_TO_HOST_SIGNAL;
    signal.param = pkt_ptr;

    /* sync_data_length  (3rd byte)+ opcode (2) + length field(1) 
     */

    signal.length = (UINT16)(sync_data_length + 2 + 1);

	RT_BT_LOG(GRAY, HCI_TD_API_368, 1, signal.length);

    OS_SEND_SIGNAL_TO_TASK(hci_td_task_handle,signal);
	TD_INF(SCO_DATA_SENT_TO_HCI_TD_TASK_PARAM_LENGHT_CH,2,sync_data_length,conn_handle);

    return API_SUCCESS;
}
#endif /* SCO_OVER_HCI | COMPILE_ESCO */

