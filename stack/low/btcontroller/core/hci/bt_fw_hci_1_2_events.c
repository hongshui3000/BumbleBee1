/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  HCI events handlers related
 *  to the Bluetooth Version 1.2 - 2.1 Features.
 */
/********************************* Logger *************************/ 
enum { __FILE_NUM__= 5 };
/********************************* Logger *************************/
#include "mem.h"

/* Includes */
#include "bt_fw_types.h"
#include "bt_fw_hci_defines.h"
#include "bt_fw_hci.h"
#include "bt_fw_hci_1_2_cmds.h"
#include "bt_fw_hci_internal.h"
#include "bz_debug.h"
#include "dma_usb.h" 
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
UINT16 g_handle = 0x00;
#endif
#ifdef COMPILE_ESCO
extern LMP_ESCO_CONNECTION_ENTITY
       lmp_esco_connection_entity[LMP_MAX_ESCO_CONN_ENTITIES];
#endif /* COMPILE_ESCO */

#if defined(ENABLE_SCO) || defined(COMPILE_ESCO)

#define HCI_SYNC_CONN_COMPLETE_EVENT_LEN        17

/**
 * Generates a Synchronous connection complete event. 
 *
 * \param ce_index      connection entity index.
 * \param link_type     SCO_LINK or ESCO_LINK.
 * \param sync_ce_index connection entity index of the esco link.
 * \param status        status of the parameter negitiation.
 *
 * \return API_SUCCESS or API_FAILURE.
 *
 */
API_RESULT hci_generate_synchronous_conn_complete_event(UINT16 ce_index,
        UINT16 sync_ce_index, HCI_LINK_TYPE link_type, UCHAR status)
{
    UCHAR event_parameter[HCI_SYNC_CONN_COMPLETE_EVENT_LEN];
    LMP_CONNECTION_ENTITY* ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    /* Populate the event structure */
    event_parameter[0] = status;
    event_parameter[1] = LSB(ce_ptr->connection_type.connection_handle);
    event_parameter[2] = MSB(ce_ptr->connection_type.connection_handle);
    memcpy(&event_parameter[3], ce_ptr->bd_addr, LMP_BD_ADDR_SIZE);
    event_parameter[9] = (UCHAR)link_type;
    memset(&event_parameter[10], 0, 1+1+2+2+1);


    if (status == HCI_COMMAND_SUCCEEDED)
    {
        switch(link_type)
        {
#ifdef ENABLE_SCO
            case SCO_LINK:
            {
                LMP_SCO_CONNECTION_DATA* sco_ce_ptr;
                sco_ce_ptr = &lmp_sco_connection_data[sync_ce_index];
                event_parameter[1] = LSB(sco_ce_ptr->sco_conn_handle);
                event_parameter[2] = MSB(sco_ce_ptr->sco_conn_handle);
                event_parameter[16] = sco_ce_ptr->air_mode;
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
                g_handle = sco_ce_ptr->sco_conn_handle;
                RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,g_handle);
#endif                
                break;
            }
#endif /* ENABLE_SCO */

#ifdef COMPILE_ESCO
            case ESCO_LINK:
            {
                LMP_ESCO_CONNECTION_ENTITY* esco_ce_ptr;
                UINT16 rx_pkt_length, tx_pkt_length;
                esco_ce_ptr = &lmp_esco_connection_entity[sync_ce_index];
                if(ce_ptr->remote_dev_role == MASTER)
                {
                    rx_pkt_length = esco_ce_ptr->m_to_s_packet_length;
                    tx_pkt_length = esco_ce_ptr->s_to_m_packet_length;
                }
                else
                {
                    rx_pkt_length = esco_ce_ptr->s_to_m_packet_length;
                    tx_pkt_length = esco_ce_ptr->m_to_s_packet_length;
                }
                event_parameter[1] = LSB(esco_ce_ptr->conn_handle);
                event_parameter[2] = MSB(esco_ce_ptr->conn_handle);
                event_parameter[10] = esco_ce_ptr->tesco;
                event_parameter[11] = esco_ce_ptr->wesco;
                event_parameter[12] = LSB(rx_pkt_length);
                event_parameter[13] = MSB(rx_pkt_length);
                event_parameter[14] = LSB(tx_pkt_length);
                event_parameter[15] = MSB(tx_pkt_length);
                event_parameter[16] = esco_ce_ptr->air_mode;
#ifdef _DAPE_TEST_GEN_FAKE_ESCO_DATA
                g_handle = esco_ce_ptr->conn_handle;
                RT_BT_LOG(GREEN, DAPE_TEST_LOG293, 1,g_handle);
#endif
                
                break;
            }
#endif /* COMPILE_ESCO */

            default:
                BZ_ASSERT(0, "link_type parameter is wrong");
                break;
        }
    } /* end if (status == HCI_COMMAND_SUCCEEDED) */

    hci_generate_event(HCI_SYNCHRONOUS_CONNECTION_COMPLETE_EVENT,
            event_parameter, HCI_SYNC_CONN_COMPLETE_EVENT_LEN);
    
    return API_SUCCESS;
}
#endif /* defined(ENABLE_SCO) || defined(COMPILE_ESCO) */

#ifdef COMPILE_ESCO
/**
 * Generates a Synchronous connection change event. 
 *
 * \param status         Connection reconfiguration status.
 * \param esco_ce_index  Connection entity index of the esco link.
 *
 * \return API_SUCCESS or API_FAILURE.
 *
 */
API_RESULT hci_generate_synchronous_conn_changed_event(UCHAR status,
        UINT16 esco_ce_index)
{
    UCHAR event_parameter[9];
    UINT16 rx_pkt_length = 0, tx_pkt_length = 0;
    UCHAR Tesco = 0,Wesco = 0;
    LMP_ESCO_CONNECTION_ENTITY *esco_ptr;

	ESCO_INF(GENERATING_SYNCHRONOUS_CONNECTION_CHANGED_EVENT,0,0);

    esco_ptr = &lmp_esco_connection_entity[esco_ce_index];

    event_parameter[0] = status;
    event_parameter[1] = LSB(esco_ptr->conn_handle);
    event_parameter[2] = MSB(esco_ptr->conn_handle);

    if (status == HCI_COMMAND_SUCCEEDED)  
    {
        if(lmp_connection_entity[esco_ptr->ce_index].remote_dev_role == MASTER)
        {
            rx_pkt_length = esco_ptr->m_to_s_packet_length;
            tx_pkt_length = esco_ptr->s_to_m_packet_length;
        }
        else
        {
            rx_pkt_length = esco_ptr->s_to_m_packet_length;
            tx_pkt_length = esco_ptr->m_to_s_packet_length;
        }
        Tesco = esco_ptr->tesco;
        Wesco = esco_ptr->wesco;
    }

	ESCO_INF(GENERATING_SYNC_CON_CHANGED_EVENT,0,0);

    event_parameter[3] = Tesco;
    event_parameter[4] = Wesco;
    event_parameter[5] = LSB(rx_pkt_length);
    event_parameter[6] = MSB(rx_pkt_length);
    event_parameter[7] = LSB(tx_pkt_length);
    event_parameter[8] = MSB(tx_pkt_length);

    hci_generate_event(HCI_SYNCHRONOUS_CONNECTION_CHANGED_EVENT,
            event_parameter, 9);

    return API_SUCCESS;
}

#endif /* COMPILE_ESCO */

#ifdef COMPILE_FEATURE_REQ_EXT
/**
 * Generates a Remote extended features complete event. 
 *
 * \param connection_handle     ACL connection handle..
 * \param page       Page number of the features queried.
 * \param max_supported_page   The max no of supported pages.
 * \param remote_features  Pointer to the features sup by remote device.
 *
 * \return None.
 *
 */
void hci_generate_remote_ext_features_complete_event(UCHAR status,
        UINT16 connection_handle, UCHAR page, UCHAR max_supported_page,
        UCHAR* remote_features)
{
    UCHAR event_parameter[13];
    event_parameter[0] = status;
    event_parameter[1] = LSB(connection_handle);
    event_parameter[2] = MSB(connection_handle);
    event_parameter[3] = page;
    event_parameter[4] = max_supported_page;
    memcpy(&event_parameter[5], remote_features, LMP_FEATURES_SIZE);
    hci_generate_event(HCI_READ_REMOTE_EXT_FEATURES_COMPLETE_EVENT,
            event_parameter, 13);
}
#endif /* COMPILE_FEATURE_REQ_EXT */

