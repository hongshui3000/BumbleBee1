/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/
enum { __FILE_NUM__= 125 };


#include "pta.h"
#include "lmp.h"
#include "bb_driver.h"
#include "mailbox.h"
#include "dma_usb.h"
#include "timer.h"

#ifdef _ENABLE_RTK_PTA_

#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
PTA_MANAGE_S_TYPE pta_manage;
PTA_TDMA_MANAGE_S_TYPE pta_tdma_manage;
#endif
//For PTA TDMA timer state
UINT32 pta_counter;
UINT8 tdma_state;
UINT8 traditional_tdma;

#ifdef _ROM_CODE_PATCHED_
//PF_ROM_CODE_PATCH_FUNC rcp_pta_profile_manage = NULL;
#endif

/**
 * Function     : pta_init
 *
 * Description  : This function is used to initial PTA setting
 *
 * Parameters  :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_init(void)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 i;

    //Initial PTA Profile manage
    pta_manage.link_num = 0;

    for (i=0;i<MAX_RECORD_PROFILE;i++)
    {
        pta_manage.profile_info[i].profile = 0;
        pta_manage.profile_info[i].con_handle = 0;
        pta_manage.profile_info[i].status = 0;
        pta_manage.profile_info[i].max_transfer_time = 0;
    }

    //Initial TDMA Manage
    pta_tdma_manage.bt_request_active_time = 0;
    pta_tdma_manage.wlan_active_time = 32;//ms
    pta_tdma_manage.bt_max_active_time = 12;//ms
    pta_tdma_manage.tdma_para.d32 = 0;
#endif

    tdma_state = 0;
    pta_counter = 0;
    traditional_tdma = 0;

    pta_main_switch(ON);
}

/**
 * Function     : pta_profile_manage
 *
 * Description  : This function is used to manage PTA profile information
 *
 * Parameters  : parameter point with HCI command parameters
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
BOOLEAN pta_profile_manage(UINT8 *parameter)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 link_num = pta_manage.link_num;
    UINT8 remove_index = 0, find_inex = 0;
    PTA_PROFILE_RECROD_S_TYPE temp_profile;
    EFUSE_POW_PARA_S_TYPE efuse_pow_para;
    efuse_pow_para.d16 = otp_str_data.power_seq_param;

#ifdef _ROM_CODE_PATCHED_
    /* could share another patch function */
    //if (rcp_pta_profile_manage != NULL)
    //{
    //   rcp_pta_profile_manage((void *)parameter);
    //    return TRUE;
    //}
#endif

    temp_profile.profile = parameter[0];
    temp_profile.status = parameter[3];
    temp_profile.con_handle = parameter[1] | (parameter[2] << 8);

    //HCI send wrong parameter
    if ((!temp_profile.status) && (!link_num))
        return FALSE;

    if (temp_profile.status)
    {
        find_inex = pta_search_available_index();
        //Can't find the correct index
        if (remove_index == 0xFF)
            return FALSE;

        pta_manage.link_num++;
        pta_manage.profile_info[find_inex].profile = temp_profile.profile;
        pta_manage.profile_info[find_inex].status = temp_profile.status;
        pta_manage.profile_info[find_inex].con_handle= temp_profile.con_handle;

        pta_profile_handle(find_inex, ON);
        //Notify wifi bt role
        pta_scan_bt_controller_role();

    }
    else
    {
        remove_index = pta_search_exist_con_handle(temp_profile.con_handle);

        //Can't find the correct index
        if (remove_index == 0xFF)
            return FALSE;

        pta_profile_handle(find_inex, OFF);

        pta_manage.link_num--;
        pta_manage.profile_info[find_inex].profile = 0;
        pta_manage.profile_info[find_inex].status = 0;
        pta_manage.profile_info[find_inex].con_handle= 0;
        pta_manage.profile_info[find_inex].max_transfer_time = 0;

    }
#endif
    return TRUE;
}

void pta_main_switch(UINT8 is_on)
{
    UINT16 pta_setting;
    //EFUSE_POW_PARA_S_TYPE efuse_pow_para;

    if (is_on)
    {
        //Enable ignore wlan_act
#ifndef _RTL8821A_
        pta_setting = 0x220;
#else
        pta_setting = otp_str_data.pta_default_setting;
#endif

        //Enable RTK mode
        BB_write_baseband_register(0x1A6, pta_setting);
        //Mask all interrupt
        BB_write_baseband_register(0x1B2, 0xFF);
        //After hw send the report and wait this time,
        //hw start detect WLAN_ACT.
        BB_write_baseband_register(0xFA, 0x32);
        //Bluewix PTA enable
        BB_write_baseband_register(0xF4, 0x01);
        //Set HW sample rate for WLAN report
        BB_write_baseband_register(0x1AA, 0x03);
        //Enable RTK co-exist mode; Note: No use
        //VENDOR_WRITE(0x14C, 0x400);

#ifdef _CCH_RETENTION_FLOW_FOR_DLPS_
        if(sleep_mode_param.dlps_restore_flow == FALSE)
#endif
        {
            RT_BT_LOG(YELLOW, ENABLE_TDMA_MODE, 0, 0);
        }

    }
    else
    {
        //Enable RTK mode
        BB_write_baseband_register(0x1A6, 0x0);
        //Bluewix PTA disable
        BB_write_baseband_register(0xF4, 0x00);
    }
}

/**
 * Function     : pta_search_exist_con_handle
 *
 * Description  : This function is used to search connection handle
 *
 * Parameters  : con_handle: connection handle
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
UINT8 pta_search_exist_con_handle(UINT16 con_handle)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 i, find_index = 0xFF;
    for (i=0;i<MAX_RECORD_PROFILE; i++)
    {
        if (pta_manage.profile_info[i].con_handle == con_handle)
        {
            find_index = i;
            break;
        }
    }

    return find_index;
#else
    return 0xFF;
#endif
}

/**
 * Function     : pta_search_available_index
 *
 * Description  : This function is used to search available index from PTA manager
 *
 * Parameters  :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
UINT8 pta_search_available_index(void)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 i;
    UINT8 find_index = 0;

    for (i=0; i<MAX_RECORD_PROFILE; i++)
    {
        if (!pta_manage.profile_info[i].profile)
        {
            find_index = i;
            break;
        }
    }

    return find_index;
#else
    return 0;
#endif
}

/**
 * Function     : pta_profile_handle
 *
 * Description  : This function is used to handle different profile
 *
 * Parameters : handle_index is manager index; is_on is that profile handle is on or off
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_profile_handle(UINT8 handle_index, UINT8 is_on)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT16 ce_index;

    //DEF_CRITICAL_SECTION_STORAGE;

    //update maximum trasfer time
    if (is_on)
    {
        if (pta_manage.profile_info[handle_index].profile < 2)
        {
            if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(
                (UINT16)pta_manage.profile_info[handle_index].con_handle,&ce_index)
                != API_FAILURE)
            {
                pta_manage.profile_info[handle_index].max_transfer_time =
                lmp_connection_entity[ce_index].flush_timeout;
            }
            else
            {
                RT_BT_LOG(RED,PTA_GET_LMP_CE_INDEX_ERROR,0,0);
            }
        }
        else
        {
            pta_manage.profile_info[handle_index].max_transfer_time =
            otp_str_data.default_max_flush_time;

        }
    }

    //handle different profile setting
    switch (pta_manage.profile_info[handle_index].profile)
    {
        case HID_PROFILE:
            if (is_on && (pta_manage.link_num == 1))
            {
                BB_write_baseband_register(0x19E, 0x100);
            }
            else
            {
                BB_write_baseband_register(0x19E, 0x000);
            }
            break;
        case A2DP_PROFILE:
/*            if (!is_on)
            {
                MINT_OS_ENTER_CRITICAL();
                //disable tdma timer flag
                tdma_state = 0;
                MINT_OS_EXIT_CRITICAL();
            }
*/
            break;
        case FTP_PROFILE:
            break;
        case PAN_PROFILE:
            break;
        case HANDSFREE_PROFILE:
            break;
        case HANDSET_PROFILE:
            break;
        default:
            break;
    }
#endif
}

/**
 * Function     : pta_get_bt_transfer_time
 *
 * Description  : This function is used to get bt transfer time
 *
 * Parameters :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_get_bt_transfer_time()
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 i;
    UINT8 first_time = 0;
    UINT32 min_t = 0;
    UINT32 min_temp_value = 0;

    if (pta_manage.link_num)
    {
        for (i=0; i<MAX_RECORD_PROFILE; i++)
        {
            if (pta_manage.profile_info[i].status)
            {
                min_temp_value = lc_get_expect_transfer_time(
                                        pta_manage.profile_info[i].con_handle,
                                        pta_tdma_manage.tdma_para.b.bt_retry_index,
                                        pta_tdma_manage.tdma_para.b.bt_pkt_type);

                //from data queue, to get transfer time.
                if (min_temp_value > 0)
                {
                    if (!first_time)
                    {
                        first_time++;
                        min_t = min_temp_value;
                    }
                    else
                    {
                        min_t = MIN(min_t, min_temp_value);
                    }
                }
            }
        }
        pta_tdma_manage.bt_request_active_time = min_t*1000;
    }
#endif
}

/**
 * Function     : pta_execule_tdma_procedure
 *
 * Description  : This function is used to PTA TDMA mode callback function
 *
 * Parameters :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_execule_tdma_procedure()
{
    //RT_BT_LOG(YELLOW, TRIGGER_TDMA_CALLBACK, 0, 0);
    pta_get_bt_transfer_time();
    pta_notify_wifi();
}

/**
 * Function     : pta_notify_wifi
 *
 * Description  : This function is used to notify wifi for TDMA mode
 *
 * Parameters :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_notify_wifi()
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 mailbox_in[8];
    UINT32 temp_time = 0;
    UINT32 max_time = pta_tdma_manage.bt_max_active_time*1000;//us

    if (traditional_tdma)
    {
        //For taditional TDMA function
        temp_time = (max_time << 6);
    }
    else
    {
        if ((pta_tdma_manage.bt_request_active_time > max_time) && (max_time))
            temp_time = (max_time << 6);
        else
            temp_time = ((pta_tdma_manage.bt_request_active_time) << 6); //us
    }

    //RT_BT_LOG(BLUE, PTA_TBT_TIME, 1, (temp_time>>6));
    {
       temp_time = ((pta_tdma_manage.bt_max_active_time*1000) << 6); //us
       mailbox_in[0] = BT_TDMA_TIME_CMD;
       mailbox_in[1] = 4;
       mailbox_in[2] = (temp_time & 0xFF);
       mailbox_in[3] = ((temp_time >> 8) & 0xFF);
       mailbox_in[4] = ((temp_time >> 16) & 0xFF);
       mailbox_in[5] = ((temp_time >> 24) & 0xFF);

#ifdef _ENABLE_MAILBOX_
       pf_os_trigger_mailbox_task(MAILBOX_WRITE, (UINT32) *((UINT32 *)mailbox_in),
                                                  (UINT32) *((UINT32 *)(mailbox_in+4)));
#endif
    }
#endif
}

/**
 * Function     : pta_tdma_control
 *
 * Description  : This function is used to control tdma mode
 *
 * Parameters :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_tdma_control(UINT8 is_on)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT32 expred_time;

    DEF_CRITICAL_SECTION_STORAGE;

    if (is_on)
    {
#ifdef _PTA_TEMP_TEST_ENABLE_
        UINT16 ce_index = 0;

        pta_manage.link_num = 1;
        pta_manage.profile_info[0].profile = A2DP_PROFILE;
        pta_manage.profile_info[0].status = 1;
        pta_manage.profile_info[0].con_handle = lmp_connection_entity[ce_index].connection_type.connection_handle;

        RT_BT_LOG(YELLOW, CON_HANDLE_NUMBER, 1, pta_manage.profile_info[0].con_handle);

        pta_manage.profile_info[0].max_transfer_time = lmp_connection_entity[ce_index].flush_timeout;

        RT_BT_LOG(YELLOW, PTA_MAX_TRANSFER_TIME, 1, pta_manage.profile_info[0].max_transfer_time);

#endif
        //Disable Ignore WLAN_ACT function
        BB_write_baseband_register(0x1A6, 0x200);

        //At the first time, controller need to check if there are packet queued in buffer.
        pta_execule_tdma_procedure();

        MINT_OS_ENTER_CRITICAL();
        //enable tdma timer flag
        tdma_state = 1;
        //initial timer counter
        pta_counter = 0;

        expred_time = (pta_tdma_manage.wlan_active_time +
                     pta_tdma_manage.bt_max_active_time) * 1000;

        timer_on_off(TIMER2_ID, expred_time, 1);

        MINT_OS_EXIT_CRITICAL();
    }
    else
    {
        MINT_OS_ENTER_CRITICAL();
        //disable tdma timer flag
        tdma_state = 0;

        traditional_tdma = 0;

        timer_on_off(TIMER2_ID, 0, 0);

        MINT_OS_EXIT_CRITICAL();

        //Enable Ignore WLAN_ACT function
        BB_write_baseband_register(0x1A6, 0x220);
    }

    RT_BT_LOG(YELLOW, CTRL_ENABLE_TDMA_MODE, 1, is_on);
#endif
}

/**
 * Function     : pta_scan_bt_controller_role
 *
 * Description  : This function is used to get bt role
 *
 * Parameters :
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void pta_scan_bt_controller_role(void)
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 index;
    UINT8 master_num = 0;
    UINT8 slave_num = 0;
    UINT16 ce_index = 0;
    UINT8 role_status = 0;
    UINT8 mailbox_in[8];

    for (index=0; index<MAX_RECORD_PROFILE; index++)
    {
        if (pta_manage.profile_info[index].status)
        {
            if (API_FAILURE == LMP_GET_CE_INDEX_FROM_CONN_HANDLE(
                       (UINT16)pta_manage.profile_info[0].con_handle,&ce_index))
            {
                continue;
            }

            if (lmp_connection_entity[ce_index].remote_dev_role == SLAVE)
                master_num++;
            else
                slave_num++;
        }
    }

    if ((master_num)&&(slave_num))
        role_status = PTA_SCANNET_ROLE;
    else if (master_num)
        role_status = PTA_MASTER_ROLE;
    else if (slave_num)
        role_status = PTA_SLAVE_ROLE;

    //notify wifi bt role
    mailbox_in[0] = BT_ROLE_REPORT_CMD;
    mailbox_in[1] = 1;
    mailbox_in[2] = role_status;

    pf_os_trigger_mailbox_task(MAILBOX_WRITE, (UINT32) *((UINT32 *)mailbox_in),
                                               (UINT32) *((UINT32 *)(mailbox_in+4)));
#endif
}

void update_pta_manager()
{
#ifndef _DISABLE_OBSOLETE_MAILBOX_CMD_
    UINT8 i;
    UINT8 index = 0;
    LMP_CONNECTION_ENTITY* ce_ptr;

    //Initial PTA Profile manage
    pta_manage.link_num = 0;

    for (i=0;i<MAX_RECORD_PROFILE;i++)
    {
        pta_manage.profile_info[i].profile = 0;
        pta_manage.profile_info[i].con_handle = 0;
        pta_manage.profile_info[i].status = 0;
        pta_manage.profile_info[i].max_transfer_time = 0;
    }

    for (i = 0; i < LMP_MAX_CE_DATABASE_ENTRIES; i++)
    {
        ce_ptr = &lmp_connection_entity[i];
        if (ce_ptr->entity_status == ASSIGNED)
        {
            if (pta_manage.link_num < MAX_RECORD_PROFILE)
            {
                pta_manage.link_num++;
                pta_manage.profile_info[index].profile = A2DP_PROFILE;
                pta_manage.profile_info[index].status = 1;
                pta_manage.profile_info[index].con_handle = ce_ptr->connection_type.connection_handle;

                //RT_BT_LOG(YELLOW, CON_HANDLE_NUMBER, 1, pta_manage.profile_info[index].con_handle);

                pta_manage.profile_info[index].max_transfer_time = ce_ptr->flush_timeout;

                //RT_BT_LOG(YELLOW, PTA_MAX_TRANSFER_TIME, 1, pta_manage.profile_info[index].max_transfer_time);
                index++;
            }
        }
    }
#endif
}

#ifdef _RTK_PTA_TEST_
UINT16 pta_temp_value = 0;

void pta_test_in_dbg_timer_callback(void)
{
    UINT16 input_value = 0x6;
    input_value = input_value + (pta_temp_value<<8);
    BB_write_baseband_register(0x1AE, input_value);
    RT_BT_LOG(BLUE,LMP_PAYLOAD_INFO,1,input_value);
    pta_temp_value++;
}

void pta_test_in_main_func(void)
{
    BB_write_baseband_register(0x1A6, 0x200);
    //mask all interrupt
    BB_write_baseband_register(0x1B2, 0x0F);
#ifdef _ENABLE_BC_HIGH_FUN_
    BB_write_baseband_register(0x1A4, 0x800);
#endif
    //bluewix PTA enable
    BB_write_baseband_register(0xFA, 0xFF);
    BB_write_baseband_register(0xF4, 0x01);

    VENDOR_WRITE(0x14C, 0x400);
}

void pta_test_in_hci_reset_func(void)
{
    BB_write_baseband_register(0x1A6, 0x200);
    //mask all interrupt
    BB_write_baseband_register(0x1B2, 0x0F);
    //enable sco filter
//    BB_write_baseband_register(0x196, 0x100);
//    BB_write_baseband_register(0x198, 0x610);
//    BB_write_baseband_register(0x1A4, 0x100);
#ifdef _ENABLE_BC_HIGH_FUN_
    BB_write_baseband_register(0x1A4, 0x800);
#endif
    BB_write_baseband_register(0xFA, 0xFF);
    //bluewix PTA enable
    BB_write_baseband_register(0xF4, 0x01);

    VENDOR_WRITE(0x14C, 0x400);
}
#endif
#endif

