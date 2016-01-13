/***************************************************************************
 Copyright (C) MindTree Consulting Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/**
 * \file
 *  Contains the functions that create/handle BT 4.1 pdus.
 * 
 */
#ifdef _SUPPORT_VER_4_1_

enum { __FILE_NUM__= 136};

/* ---------------------- Includes ------------------------------------- */
#include "lmp_internal.h"
#include "btc_defines.h"
#include "lmp_spec_defines.h"
#include "bt_fw_types.h"
#include "lmp.h"
#include "lmp_4_1.h"
#include "mem.h"
#include "bb_driver_4_1.h"
/* ---------------------- Externs -------------------------------------*/
extern UINT8 lc_sco_pause_status;
#ifdef _SUPPORT_PCA_ADJUST
BT_PCA_MANAGER_S bt_pca_manager;
#endif
#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_lmp_validate_clk_adj_param_func = NULL;
#endif

/* ----------------------Static functions----------------------------- */
/* --------------------- [Static] PDU Handlers ----------------------- */

/**
 * Handles the 4.1 pdus.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param piconet_id  Piconet ID.
 * 
 * \return BT_FW_ERROR or BT_FW_SUCCESS
 */
UCHAR lmp_handle_4_1_incoming_pdu(LMP_PDU_PKT *lmp_pdu_ptr, UINT16 ce_index)
{
    UCHAR pdu_opcode;
    UCHAR pdu_ext_opcode;
    UCHAR am_addr;

    pdu_opcode = (UCHAR)(lmp_pdu_ptr->payload_content[0] >> 1);
    pdu_ext_opcode = lmp_pdu_ptr->payload_content[1];

    //RT_BT_LOG(GRAY, LMP_HANDLE_BT3_0_PDU_MSG, 2, pdu_opcode, pdu_ext_opcode);

    am_addr = lmp_connection_entity[ce_index].am_addr;

    switch(pdu_opcode)
    {
        case LMP_ESCAPE1_OPCODE:
        case LMP_ESCAPE2_OPCODE:
        case LMP_ESCAPE3_OPCODE:
            break;

        case LMP_ESCAPE4_OPCODE:
            switch(pdu_ext_opcode)
            {
                case LMP_CLK_ADJ_OPCODE:
                        lmp_handle_lmp_clk_adj_pdu(lmp_pdu_ptr,
                                ce_index);
                        break;

                case LMP_CLK_ADJ_ACK_OPCODE:
                        lmp_handle_lmp_clk_adj_ack_pdu(lmp_pdu_ptr,
                                ce_index);
                        break;
                case LMP_CLK_ADJ_REQ_OPCODE:
                        lmp_handle_lmp_clk_adj_req_pdu(lmp_pdu_ptr,
                                ce_index);
                        break;

                default:
                        return BT_FW_ERROR;
            }
            break;
            
        default:
            return BT_FW_ERROR;
    }

    /* need to free BT4.1 req - added by austin */
   	if(OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, lmp_pdu_ptr) != BT_ERROR_OK)
   	{
   		    RT_BT_LOG(GRAY, LMP_EDTM_522, 0, 0);
   	}
    
    return BT_FW_SUCCESS;
}
#ifdef _SUPPORT_PCA_ADJUST

/**
 * Handles lmp clk adj pdu.
 * 
 * \param lmp_pdu_ptr LMP PDU packet pointer.
 * \param ce_index CE Index.
 * 
 * \return None.
 */
void lmp_handle_lmp_clk_adj_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
#ifdef _SUPPORT_PCA_ADJUST
    if(lmp_feature_data.feat_page2[0] & COARSE_CLOCK_ADJUSTMENT)
    {
        UCHAR piconet_id;
        UCHAR clk_adj_id = 0xFF;
        UINT32 instant;
        INT16 clk_adj_us;
        UINT16 clk_adj_slots;
        UINT16 clk_adj_mode;
        UINT16 err_code = 0xFF;
        UINT32 clk;

        piconet_id = lmp_connection_entity[ce_index].phy_piconet_id ;
        
        lc_get_clock_in_scatternet(&clk, piconet_id);

        /* extract parameters from lmp. */
        clk_adj_id = (UCHAR)(lmp_pdu_ptr->payload_content[2]);
        instant = ((lmp_pdu_ptr->payload_content[3])|
                  (lmp_pdu_ptr->payload_content[4]<<8)|
                  (lmp_pdu_ptr->payload_content[5]<<16)|
                  (lmp_pdu_ptr->payload_content[6]<<24));
        clk_adj_us = ((lmp_pdu_ptr->payload_content[7])|
                     (lmp_pdu_ptr->payload_content[8]<<8));
        clk_adj_slots = (lmp_pdu_ptr->payload_content[9]);
        clk_adj_mode = (lmp_pdu_ptr->payload_content[10]);

        RT_BT_LOG(BLUE, DAPE_TEST_LOG578, 8, clk, piconet_id,
        clk_adj_id,
        instant<<1, 
        clk_adj_slots, clk_adj_us,
        clk_adj_mode, bt_pca_manager.clk_adj_id);   

        if (clk_adj_id == bt_pca_manager.clk_adj_id)
        {
            err_code = INVALID_LMP_PARAMETERS_ERROR;
        }
        else
        {
            err_code = lmp_validate_clk_adj_param(clk_adj_us, clk_adj_slots, SLAVE);
        }
        if (err_code != HCI_COMMAND_SUCCEEDED)
        {
            /* we don't have to repond to requests that has wrong parameters. */
            if (err_code != INVALID_LMP_PARAMETERS_ERROR)
        {
            lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                    LMP_CLK_ADJ_OPCODE, 
                    (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr), 
                        err_code);
            }
        }
        else
        {
            UINT16 clk_cnt;
            UINT32 clk_slot;

            if (clk_adj_us < 0)
            {
                //clk_cnt = 624 + clk_adj_us;
                clk_cnt = 0 - clk_adj_us;
                clk_slot = instant + clk_adj_slots - 1;
            }
            else
            {
                clk_cnt = 624 - clk_adj_us;
                clk_slot = instant + clk_adj_slots;
            }

            BB_set_piconet_adjust(piconet_id, clk_slot, clk_cnt, instant);
            bt_pca_manager.instant = instant << 1;
            bt_pca_manager.clk_adj_slots = clk_adj_slots;
            bt_pca_manager.clk_adj_us = clk_adj_us;
            
            RT_BT_LOG(BLUE, DAPE_TEST_LOG567, 8, clk, piconet_id, instant<<1, 
            clk_adj_slots, clk_adj_us,
            clk_adj_mode, clk_slot, clk_cnt);   
     
            /* generate the response */
            lmp_generate_clk_adj_ack_pdu(ce_index, clk_adj_id);
            bt_pca_manager.clk_adj_id = clk_adj_id;
            bt_pca_manager.clk_adj_ce_index = ce_index;
        }
    }
    else /* if(lmp_feature_data.feat_page2[0]*/
#else    
    {
        lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_CLK_ADJ_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                UNSUPPORTED_REMOTE_FEATURE_ERROR);
    }
#endif
    return; 
    
}
void lmp_handle_lmp_clk_adj_ack_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    /* check if all the slaves have received the clk_adj_req*/

    UINT8 clk_adj_id;

    clk_adj_id = lmp_pdu_ptr->payload_content[2];
    if (clk_adj_id == bt_pca_manager.clk_adj_id)
    {
        bt_pca_manager.slv_pca_support_bm &= ((~BIT0) << ce_index);
        RT_BT_LOG(GREEN, DAPE_TEST_LOG574, 2, ce_index, bt_pca_manager.slv_pca_support_bm);
    }
    if (bt_pca_manager.slv_pca_support_bm == 0)
    {
        lmp_cleanup_param_after_clk_adj_procedure();
    }
    return; 
}
void lmp_handle_lmp_clk_adj_req_pdu(LMP_PDU_PKT *lmp_pdu_ptr,
        UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT16 err_code = COARSE_CLK_ADJ_REJ_BUT_WILL_TRY_TO_ADJUST_USING_DRAG;
    //RT_BT_LOG(RED, YL_DBG_HEX_2, 2, ce_index, BB_read_native_clock());
    INT16 clk_adj_us;
    UINT8 clk_adj_slots;
    UINT8 clk_adj_period;
    UINT32 clk;
    UINT8 piconet_id;

    ce_ptr = &lmp_connection_entity[ce_index];
    piconet_id = ce_ptr->phy_piconet_id;
    lc_get_clock_in_scatternet(&clk, piconet_id);

    clk_adj_us = ((lmp_pdu_ptr->payload_content[2])|
                  (lmp_pdu_ptr->payload_content[3]<<8));
    clk_adj_slots = (lmp_pdu_ptr->payload_content[4]);
    clk_adj_period = (lmp_pdu_ptr->payload_content[5]);
    
    RT_BT_LOG(BLUE, DAPE_TEST_LOG572, 7, clk, ce_index, piconet_id, ce_ptr->remote_dev_role, 
       clk_adj_slots,  clk_adj_us, clk_adj_period);
    do
    {
        
        /* If we are someone's slave, reject it. */
        if (lc_sca_manager.slave_cnt)
        {
            break;
        }
        if (bt_pca_manager.pca_updating)
        {
            err_code = DIFFERENT_TRANSACTION_COLLISION;
            break;
        }

        err_code = lmp_validate_clk_adj_param(clk_adj_us, clk_adj_slots, MASTER);
        
        if(err_code!= HCI_COMMAND_SUCCEEDED)
        {
            break;
        }
        err_code = HCI_COMMAND_SUCCEEDED;
    }
    while (0);

    if (err_code != HCI_COMMAND_SUCCEEDED)
    {
        lmp_send_lmp_not_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
                LMP_CLK_ADJ_REQ_OPCODE,
                (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr),
                err_code);
    }
    else
    {
        lmp_send_lmp_accepted_ext(ce_index, LMP_ESCAPE4_OPCODE,
            LMP_CLK_ADJ_REQ_OPCODE,
            (UCHAR)LMP_GET_TRANSACTION_ID(lmp_pdu_ptr));
        if ((clk_adj_us == 0) && (clk_adj_slots == 0))
        {
            return;
        }
        else
        {
            bt_pca_manager.slv_pca_support_bm = 0;
            UINT32 instant;
            /* Note: ">> 1" is used for transferring clk[27:0] to clk[27:1]. */
            instant = (clk + bt_pca_manager.clk_offset_for_update) >> 1; 
            RT_BT_LOG(WHITE, DAPE_TEST_LOG582, 2, bt_pca_manager.clk_offset_for_update,
                instant);
            lmp_generate_clk_adj_pdu(instant, clk_adj_slots, clk_adj_us, 0);
        }

    }
    return; 
}
/* --------------------- PDU Generaters ------------------------------ */

/**
 * Generates clk_adj pdu.
 * param 
 * 1. instant: CLKold[27:1] at the time of the coarse clock adjustment,
 *    based on the value of time_base_offset before the adjustment is made. 
 *    Only even values are valid.
 * 2. clk_adj_slots: The difference between the clocks at the adjustment
 *    instant (i.e. CLKnew[27:1] - CLKold[27:1]).
 * 3. clk_adj_us:The offset between the old and new slot boundaries.
 *    Valid range is -624 to +624.
 * 4. clk_adj_mode:0: Before Instant; 1: After Instant; 2-255: RESERVED
 *
 * \return API_FAILURE if remote device doesnot support PCA.
 */

API_RESULT lmp_generate_clk_adj_pdu(UINT32 instant, 
           UINT8 clk_adj_slots, INT16 clk_adj_us, UINT8 clk_adj_mode)
{
    //UCHAR parameter_list[LMP_CLK_ADJ_LEN];
    LMP_PDU_PKT* pdu_ptr;
    UCHAR piconet_id;
    UINT8 result = HCI_COMMAND_SUCCEEDED;
    
    //ce_ptr = &lmp_connection_entity[ce_index];
    piconet_id = lc_get_master_piconet_id();

    if (lmp_self_device_data.number_of_acl_conn == 0)
    {
        result = COMMAND_DISALLOWED_ERROR;
        //return API_FAILURE;
    }

    /* check if there is still some slaves that we have not received their 
       ack since last clk_adj_req. If bt_pca_manager.slv_pca_support_bm != 0, then
       we are waiting someone's ack and we can't send 2nd clk_adj_pdu.
       WARNING!!! this check should be done before lmp_validate_clk_adj_param();
       because in bt_pca_manager.slv_pca_support_bm may be changed in lmp_validate_clk_adj_param().
       */
    if ((bt_pca_manager.slv_pca_support_bm) || (bt_pca_manager.pca_clk_req_sent))
    {
        RT_BT_LOG(WHITE, DAPE_TEST_LOG573, 2, bt_pca_manager.clk_adj_id, bt_pca_manager.slv_pca_support_bm);
        result = COMMAND_DISALLOWED_ERROR;
    }
    else
    {
    result = lmp_validate_clk_adj_param(clk_adj_us, clk_adj_slots, MASTER);
    }

    if (result)
    {
        return result;
    }
    
    if (OS_ALLOC_BUFFER(lmp_pdu_buffer_pool_handle,(void **)(&pdu_ptr)) 
                                                            != BT_ERROR_OK)
    {
        return API_FAILURE;
    }
#ifdef _DAPE_TEST_CHK_PCA_
	RT_BT_LOG(GREEN, DAPE_TEST_LOG207, 1, BB_read_native_clock());
#endif

    /* Prepare for Broadcast Tx FIFO. Re-initial it with hci_check_and_enable_eir_trans()
       after we free the pkt. */
    bb_enable_eir(FALSE);

    pdu_ptr->payload_content[0] =
        (UCHAR)(LMP_ESCAPE4_OPCODE << 1);
    pdu_ptr->payload_content[1] = LMP_CLK_ADJ_OPCODE;
    pdu_ptr->pdu_length = 15;
    UINT32 clk = BB_read_native_clock();

#ifdef _DAPE_TEST_MANUAL_GEN_PCA_INSTANT_
    instant = (clk>>1) + 0x7D00; /* 7d00: 20sec*/
    clk_adj_us = 3;
    clk_adj_slots = 8;
    clk_adj_mode = 1;
    RT_BT_LOG(GREEN, DAPE_TEST_LOG525, 6, clk, instant, 
    clk_adj_us,piconet_id,
    BB_read_baseband_register(0x46)/*nbc*/,0);
#endif
        /*clk_adj_clk: CLK[27:2] for the moment that the PDU is transmitted.*/
#ifdef _DAPE_GEN_CLK_ADJ_CLK_BY_FW        
    UINT32 clk_adj_clk = clk + 16;
#endif
    bt_pca_manager.clk_adj_id ++;  

    pdu_ptr->payload_content[2] = bt_pca_manager.clk_adj_id;

    pdu_ptr->payload_content[3] = instant;
    pdu_ptr->payload_content[4] = instant >> 8;
    pdu_ptr->payload_content[5] = instant >> 16;
    pdu_ptr->payload_content[6] = instant >> 24;
    
    pdu_ptr->payload_content[7] = clk_adj_us;
    pdu_ptr->payload_content[8] = clk_adj_us >> 8;
    pdu_ptr->payload_content[9] = clk_adj_slots;
    pdu_ptr->payload_content[10] = clk_adj_mode;

    pdu_ptr->payload_content[11] = clk_adj_clk >> 2;
    pdu_ptr->payload_content[12] = clk_adj_clk >> 10;
    pdu_ptr->payload_content[13] = clk_adj_clk >> 18;
    pdu_ptr->payload_content[14] = clk_adj_clk >> 26;

    UINT16 clk_cnt;
    UINT32 clk_slot;
    if (clk_adj_us < 0)
    {
        clk_cnt = 0 - clk_adj_us;
        clk_slot = instant + clk_adj_slots - 1;
    }
    else
    {
        clk_cnt = 624 - clk_adj_us;
        clk_slot = instant + clk_adj_slots;
    }

    BB_set_piconet_adjust(piconet_id, clk_slot, clk_cnt, instant);
    RT_BT_LOG(GREEN, DAPE_TEST_LOG565, 9, clk, piconet_id, instant<<1, 
    clk_adj_slots, clk_adj_us,
    BB_read_baseband_register(0x46)&(0xff)/*nbc*/, clk_adj_mode,
    clk_slot<<1, clk_cnt);
    bt_pca_manager.instant = instant << 1;
    bt_pca_manager.clk_adj_slots = clk_adj_slots;
    bt_pca_manager.clk_adj_us = clk_adj_us;
    /* copy data from dmem to sram */
    memcpy(bzdma_tx_buf[piconet_id],
           pdu_ptr->payload_content, pdu_ptr->pdu_length);
    BB_prepare_clk_adj_broadcast(piconet_id);

    RT_BT_LOG(GREEN, LMP_MSG_SENT_EXT_PDU, 17, 
                    clk, pdu_ptr->payload_content[0]>>1, 
                    pdu_ptr->payload_content[1], pdu_ptr->am_addr, 
                    piconet_id, 
                    pdu_ptr->payload_content[0] & 0x01, 0xFF,
                    1,                        
                    pdu_ptr->payload_content[0],
                    pdu_ptr->payload_content[1],
                    pdu_ptr->payload_content[2],
                    pdu_ptr->payload_content[3],
                    pdu_ptr->payload_content[4],
                    pdu_ptr->payload_content[5],
                    pdu_ptr->payload_content[6],
                    pdu_ptr->payload_content[7],
                    lc_sco_pause_status);

    /* We can release this pdu since we already copy the payload into 
       bzdma_tx_buf[xx]. And bzdma_tx_buf does not need flush since everyone who
       wants to use bzdma_tx_buf[xx] just need to re-write the content. */
    OS_FREE_BUFFER(lmp_pdu_buffer_pool_handle, (void*)pdu_ptr);             
    
    bt_pca_manager.pca_updating = 1;
    bt_pca_manager.pca_clk_req_sent = 1;
    bt_pca_manager.pca_clk_req_nbc_to = 0; 
    bt_pca_manager.tpoll_cnt_after_clk_req = 0;
    return API_SUCCESS;
}
void lmp_generate_clk_adj_ack_pdu(UINT16 ce_index, UINT16 clk_adj_id)
{
    UCHAR parameter_list[LMP_CLK_ADJ_ACK_LEN];

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_CLK_ADJ_ACK_OPCODE;
    parameter_list[3] = clk_adj_id;

    lmp_generate_pdu(ce_index, parameter_list, LMP_CLK_ADJ_ACK_LEN,
            MASTER_TID, LMP_NO_STATE_CHANGE);

}
void lmp_generate_clk_adj_req_pdu(UINT16 ce_index, INT16 clk_adj_us,
    UINT8 clk_adj_slots, UINT8 clk_adj_period)
{
    UCHAR parameter_list[LMP_CLK_ADJ_REQ_LEN];
    UINT32 cur_clk;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    lc_get_clock_in_scatternet(&cur_clk, ce_ptr->phy_piconet_id);

    parameter_list[0] = LMP_ESCAPE4_OPCODE;
    parameter_list[2] = LMP_CLK_ADJ_REQ_OPCODE;
    parameter_list[3] = clk_adj_us;
    parameter_list[4] = clk_adj_us >> 8;    
    parameter_list[5] = clk_adj_slots;
    parameter_list[6] = clk_adj_period;

    RT_BT_LOG(GREEN, DAPE_TEST_LOG568, 6, cur_clk, ce_ptr->phy_piconet_id,
    ce_index, clk_adj_slots, clk_adj_us, clk_adj_period);

    lmp_generate_pdu(ce_index, parameter_list, LMP_CLK_ADJ_REQ_LEN,
            MASTER_TID, LMP_NO_STATE_CHANGE);

}
UINT8 lmp_validate_clk_adj_param(INT16 clk_adj_us, UINT16 clk_adj_slots, UINT8 self_role)
{
#ifdef _ROM_CODE_PATCHED_
    if (rcp_lmp_validate_clk_adj_param_func != NULL)
    {
        UINT8 result;
        if (rcp_lmp_validate_clk_adj_param_func((void*)&clk_adj_us, &clk_adj_slots, &self_role, &result))
        {
            return result;
        }
    }
#endif

    UINT8 result = INVALID_LMP_PARAMETERS_ERROR;
    UINT8 i;
    do
    {
        /* Role switch is on progress. Reject PCA. */
        if (lmp_mss_state != LMP_MSS_INIT)
        {
            result = DIFFERENT_TRANSACTION_COLLISION;
            break;
        }

        /* Local feature not support PCA. */
        if(((lmp_feature_data.feat_page2[0] & COARSE_CLOCK_ADJUSTMENT) == 0) ||
            ((lmp_feature_data.feat_page2[0] & CSB_MASTER) == 0) ||
            ((lmp_feature_data.feat_page2[0] & SYNCHRONIZATION_TRAIN_MASTER) == 0))
        {
            RT_BT_LOG(RED, DAPE_TEST_LOG564, 3, 
            (lmp_feature_data.feat_page2[0] & COARSE_CLOCK_ADJUSTMENT),
            (lmp_feature_data.feat_page2[0] & CSB_MASTER),
            (lmp_feature_data.feat_page2[0] & SYNCHRONIZATION_TRAIN_MASTER));
        
            result = UNSUPPORTED_FEATURE_OR_PARAMETER_ERROR;
            break;
        }

        /* Check if remote feature support PCA. */
        for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES;  i ++)
        {
            LMP_CONNECTION_ENTITY *tmp_ce_ptr;

            tmp_ce_ptr = &lmp_connection_entity[i];
            if (tmp_ce_ptr->entity_status == ASSIGNED)
            {
                if (((lmp_connection_entity[i].features[2][0] & COARSE_CLOCK_ADJUSTMENT)== 0) ||
                    ((lmp_connection_entity[i].features[2][0] & CSB_SLAVE) == 0) || 
                    ((lmp_connection_entity[i].features[2][0] & SYNCHRONIZATION_TRAIN_SCAN_SLAVE) == 0) ||
#ifdef _DAPE_TEST_PCA_FAKE_WRONG
                (tmp_ce_ptr->remote_dev_role == MASTER)

#else
                (tmp_ce_ptr->remote_dev_role == self_role)
#endif                

                )
                {
                    RT_BT_LOG(RED, DAPE_TEST_LOG571, 5, i, 
                        tmp_ce_ptr->remote_dev_role,
                    (lmp_connection_entity[i].features[2][0] & COARSE_CLOCK_ADJUSTMENT),
                    (lmp_connection_entity[i].features[2][0] & CSB_SLAVE),
                    (lmp_connection_entity[i].features[2][0] & SYNCHRONIZATION_TRAIN_SCAN_SLAVE));
                
                    result = UNSUPPORTED_REMOTE_FEATURE_ERROR;
                    bt_pca_manager.slv_pca_support_bm = 0;
                    break;
                }
                else
                {
                    bt_pca_manager.slv_pca_support_bm |= (BIT0 << i);
                }
            }
        }
        if (result == UNSUPPORTED_REMOTE_FEATURE_ERROR)
        {
            break;
        }
        /* Parameter Error. */
        if ((clk_adj_us > 624) || (clk_adj_us < -624))
        {
            break;
        }
        if (clk_adj_slots > 255)
        {
            break;
        }
        result = HCI_COMMAND_SUCCEEDED;
    }
    while (0);
//RT_BT_LOG(RED, DAPE_TEST_LOG207, 1, result);
    return result;
}
void lmp_cleanup_param_after_clk_adj_procedure(void)
{
    BB_write_baseband_register(PICONET_LOOK_UP_TABLE_BASE_ADDR, LC_INVALID_PACKET_TYPE);

    bt_pca_manager.pca_clk_req_sent = 0;
    bt_pca_manager.pca_clk_req_nbc_to = 0; 
    bt_pca_manager.tpoll_cnt_after_clk_req = 0;
    hci_check_and_enable_eir_trans();     
    
    RT_BT_LOG(WHITE, DAPE_TEST_LOG577, 2, BB_read_native_clock(), bt_pca_manager.clk_adj_id);
}
#endif  /* _SUPPORT_PCA_ADJUST */
#endif /* _SUPPORT_VER_4_1_ */
