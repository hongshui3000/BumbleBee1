/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file
 *  Contains inquiry scan and page scan procedures for both 1.1 and 1.2 scans
 *  (interlaced scanning, first FHS).
 */

/********************************* Logger *************************/ 
enum { __FILE_NUM__= 41 };
/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "bt_fw_os.h"
#include "lc_internal.h"
#include "btc_defines.h"
#include "bt_fw_globals.h"
#include "lc.h"
#include "gpio.h"
#include "lmp_internal.h"

#ifdef _3DD_FUNCTION_SUPPORT_
#include "bt_3dd.h"
#endif

/* ===================== Variable Declaration Section ===================== */
#define PAGE_SCAN_INTERLACED      0x0800
#define INQ_SCAN_INTERLACED       0x1000

UCHAR lc_pagescan_piconet_id = SCA_PICONET_INVALID;
UCHAR lc_paging_piconet_id = SCA_PICONET_INVALID;

extern UCHAR g_lc_scan_slot_timer_in_use;

/* ================== Static Function Prototypes Section ================== */

/* ================== For rom code patch function point ================== */

#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
PF_ROM_CODE_PATCH_FUNC rcp_lc_start_write_scan_mode = NULL;
#endif

PF_ROM_CODE_PATCH_FUNC rcp_lc_retrieve_page_scan_func = NULL;
#endif


/* ===================== Function Definition Section ====================== */
/** 
 * Programs scan mode to the baseband. Also checks if interlaced scan 
 * is enabled by the host and programs it to the baseband.
 * 
 * \param scan_enable The value of scan-enable parameter that the 
 *                    host has written last.
 * 
 * \return None.
 */
void lc_start_write_scan_mode(UCHAR scan_enable)
{
    lc_write_scan_mode(scan_enable);
    lc_check_and_enable_scans_in_scatternet();

    return;
}


void lc_write_scan_mode(UCHAR scan_enable)
{
    UINT16 read;
    UINT32 lap ;
    UCHAR parity_bits[5];
    UINT16 lap_index ;
#ifdef _DAPE_CHK_WHCK_EIR
    lmp_self_device_data.page_scan_interval = 0x200;
    lmp_self_device_data.page_scan_window = 0x12;
    /* (dape 20131018) do not make inq scan interval too short which will
    result in failure in WHCK P2P hct2.4.01.*/
    lmp_self_device_data.inquiry_scan_interval = 0x600;
    lmp_self_device_data.inquiry_scan_window = 0x12;
    lmp_self_device_data.inquiry_scan_type = HCI_INTERLACED_INQ_SCAN;            
    RT_BT_LOG(WHITE, DAPE_TEST_LOG293, 1, 8888);
#endif

    if (g_lc_scan_slot_timer_in_use == 0x1)
    {
        /* Scan timer in use, will be restart scan after the use */
        return;
    }

#ifdef _CCH_RTL8723A_B_CUT
    lmp_update_lps_para();
#endif

	if(scan_enable == 0)
	{
		lc_kill_scan_mode();
		return;
	}
    
#ifdef _ROM_CODE_PATCHED_
#ifdef _CCH_8821_RCP_
    if (rcp_lc_start_write_scan_mode != NULL)
    {
        if ( rcp_lc_start_write_scan_mode((void*)(&scan_enable)) )
        {
            return;
        }
    }     
#endif
#endif
    
    /* Donot allow scan if already inquiry or page procedure is in progress */
	/* dape: We don't check if device is in inquiry or page state, we leave it
	   in the last step: lc_check_and_enable_scans_in_scatternet(). Because we
	   have to set page_scan_interval back to right value after role switch even
	   if the device is in inquiry or page state. */
#ifndef _DAPE_TEST_STILL_SET_SCAN_WINDOW_INTERVAL
    /* Start Inquiry Scan */ 
    if (scan_enable & 0x01)
    {
        if (lmp_self_device_data.lc_cur_dev_state & LC_CUR_STATE_INQUIRY)
        {
            return;
        }
    }

    /* Start Page Scan */
    if(scan_enable & 0x02)
    {
        if (lmp_self_device_data.lc_cur_dev_state & LC_CUR_STATE_PAGE)
        {
            return;
        }
    }
#endif    

    /* Check for interlaced scanning , if interlaced scan is enabled by the 
     * host and the scan windows allow interlaced scanning 
     */
    if((lmp_self_device_data.inquiry_scan_type == HCI_INTERLACED_INQ_SCAN) && 
        (lmp_self_device_data.inquiry_scan_interval > 
        (lmp_self_device_data.inquiry_scan_window << 1)))
    {
        lmp_self_device_data.interlaced_inq_scan = TRUE;
    }
    else
    {
#ifdef _3DD_FUNCTION_SUPPORT_
#ifdef _SUPPORT_CSB_TRANSMITTER_
        if (!bt_3dd_var.is_csb_mode)
#endif
        {
            /* check for 3dd extension inquiry scan */
            if((lmp_self_device_data.inquiry_scan_type == HCI_3DD_INQ_SCAN) && 
                (lmp_self_device_data.inquiry_scan_interval > 
                (lmp_self_device_data.inquiry_scan_window << 1)))
            {
                lmp_self_device_data.interlaced_inq_scan = TRUE;
            }
        }
        else
#endif
        {
            lmp_self_device_data.interlaced_inq_scan = FALSE;    
        }
    }

    if((lmp_self_device_data.page_scan_type == HCI_INTERLACED_PAGE_SCAN) && 
        (lmp_self_device_data.page_scan_interval > 
        (lmp_self_device_data.page_scan_window << 1)))
    {
        lmp_self_device_data.interlaced_page_scan = TRUE;
    }
    else
    {
        lmp_self_device_data.interlaced_page_scan = FALSE;
    }

    lap_index = 0;

	/* Extract the LAP */
    lap = (lmp_self_device_data.iac_lap[lap_index][2] << 16) |
          (lmp_self_device_data.iac_lap[lap_index][1] << 8) |
          (lmp_self_device_data.iac_lap[lap_index][0]);

    /* Configure inquiry scan giac_diac_registers */
    BB_write_baseband_register(GIAC_DAIC_REGISTER1, lap & 0xFFFF);
    BB_write_baseband_register_lower_octet(GIAC_DAIC_REGISTER2, lap >> 16);

    /* Generate parity bits corresponding to lap */
	lc_generate_parity_bits(lap, &parity_bits[0]);

    /* Configure Calculated value of Parity bits */
    BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER1,
            (*((UINT16 *) &parity_bits[0])));
    BB_write_baseband_register(INQUIRY_PARITY_BITS_REGISTER2,
            (*((UINT16 *) &parity_bits[2])));
    BB_WRITE_INQ_PARITY_BITS((*((UINT16 *) &parity_bits[4])));

    /* Program FHS parameter register */
    read = (lmp_self_device_data.page_scan_repetition_mode << 6) |
           (0x02 << 4) |
           ((lmp_self_device_data.page_scan_type & 0x01) << 3) |
           (lmp_self_device_data.page_scan_mode & 0x07);
    BB_write_baseband_register_upper_octet(FHS_PARAMETER_REGISTER, read);

    /* Start Inquiry Scan */    
    if(scan_enable & 0x01)
    {
        /* Program ISW */
        BZ_REG_S_INQ_SCAN_WINDOW isw;
		
        *(UINT16*)&isw = BB_read_baseband_register(INQ_SCAN_WINDOW_REGISTER);

        /* lmp_self_device_data.inquiry_scan_window has to be even. */
        isw.inq_scan_window = 
           (((lmp_self_device_data.inquiry_scan_window + 2) + 1) >> 1) & 0xfff;

#ifdef _CCH_INQ_PAG_SCAN_LEG_
        UINT16 temp_inq_scan_window;
        UINT16 SCO_packet_type;
        UCHAR number_of_sco_connection;

        temp_inq_scan_window = isw.inq_scan_window;
        SCO_packet_type = lmp_self_device_data.sco_pkt_type;		
        number_of_sco_connection = 
		lmp_self_device_data.total_no_of_sco_conn + lmp_self_device_data.number_of_esco_connections;

        switch(number_of_sco_connection)
        {
            case 0 : 
                temp_inq_scan_window = isw.inq_scan_window;
                break;

            case 1 :
                if ( (lmp_self_device_data.number_of_esco_connections > 0) || (SCO_packet_type == HV3) )
                {  /* SCO and Esco are not allowed together. */
                    temp_inq_scan_window = (isw.inq_scan_window) << 1;
                }
                else if (SCO_packet_type == HV2)
                {
                    temp_inq_scan_window = (isw.inq_scan_window) << 2;
                }
                break;

             case 2 : 
                temp_inq_scan_window = (isw.inq_scan_window) << 2;
                 break;

             default :
                 temp_inq_scan_window = (isw.inq_scan_window) << 2;
         }	 

         if( temp_inq_scan_window & 0xf000 )  // temp_page_scan_window larger than page_scan_window size
         {
             temp_inq_scan_window = 0xfff; 
         }

         isw.inq_scan_window = temp_inq_scan_window;	
		 
#endif

        /* Program interlaced scan */
        if (lmp_self_device_data.interlaced_inq_scan == TRUE)
        {
            isw.scan_type = 1;
        }
        else
        {
            isw.scan_type = 0;
        }
        

        
        BB_write_baseband_register(INQ_SCAN_WINDOW_REGISTER, *(UINT16*)&isw);

        /* Program ISI */
        BB_write_baseband_register(INQ_SCAN_INTERVAL_REGISTER, 
            (UINT16) ((lmp_self_device_data.inquiry_scan_interval + 1) >> 1));
	}

    /* Start Page Scan */
    if(scan_enable & 0x02)
    {
        /* Program PSW */
        BZ_REG_S_PAGE_SCAN_WINDOW psw;
        *(UINT16*)&psw = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
        psw.page_scan_window =
            (( lmp_self_device_data.page_scan_window + 2 + 1) >> 1) & 0xfff;


#ifdef _CCH_INQ_PAG_SCAN_LEG_
        UINT16 temp_page_scan_window;
        UINT16 SCO_packet_type;
        UCHAR number_of_sco_connection;


        temp_page_scan_window = psw.page_scan_window;
        SCO_packet_type = lmp_self_device_data.sco_pkt_type;		
        number_of_sco_connection = 
		lmp_self_device_data.total_no_of_sco_conn + lmp_self_device_data.number_of_esco_connections;

        switch(number_of_sco_connection)
        {
            case 0 : 
                temp_page_scan_window = psw.page_scan_window;
                break;

            case 1 :
                if ( (lmp_self_device_data.number_of_esco_connections > 0) || (SCO_packet_type == HV3) )
                {  /* SCO and Esco are not allowed together. */
                    temp_page_scan_window = (psw.page_scan_window) << 1;
                }
                else if (SCO_packet_type == HV2)
                {
                    temp_page_scan_window = (psw.page_scan_window) << 2;
                }
                break;

             case 2 : 
                temp_page_scan_window = (psw.page_scan_window) << 2;
                 break;

             default :
                 temp_page_scan_window = (psw.page_scan_window) << 2;
         }	 

         if( temp_page_scan_window & 0xf000 )  // temp_page_scan_window larger than page_scan_window size
         {
             temp_page_scan_window = 0xfff; 
         }

         psw.page_scan_window =	temp_page_scan_window;	
		 
#endif
		
		
        BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, *(UINT16*)&psw);

        /* Program PSI */
        BB_write_baseband_register(PAGE_SCAN_INTERVAL_REGISTER, 
                     (UINT16) ((lmp_self_device_data.page_scan_interval + 1) >> 1) );

        /* Program interlaced scan */
        if (lmp_self_device_data.interlaced_page_scan == TRUE)
        {
            OR_val_with_bb_reg(FHS_PARAMETER_REGISTER, PAGE_SCAN_INTERLACED);
        }
        else
        {
            AND_val_with_bb_reg(FHS_PARAMETER_REGISTER, 
                            (UINT16) (~PAGE_SCAN_INTERLACED) );
        }
    }

#if 0
	/* Donot allow scan if already inquiry or page procedure is in progress */
	if((lmp_self_device_data.lc_cur_dev_state & 
		(LC_CUR_STATE_PAGE | LC_CUR_STATE_INQUIRY)) != 0)
	{
		return;
	}
#endif
}

/** 
 * Handles FHS packet sent interrupt. Random backoff timer is started here.
 * 
 * \param None.
 * 
 * \return None.
 */
void lc_handle_inq_fhs_sent_intr(void)
{
    UCHAR status;
    UINT16 random_number;
    UINT16 max_rand = MAX_RAND;
    
#ifdef ENABLE_LOGGER_LEVEL_2
	LC_LOG_INFO(LOG_LEVEL_LOW,FHS_SENT_INTR,0,0);
#endif

    /* scanning intervals < 1.28s. Max Rand may be as small as 127 */
    if(lmp_self_device_data.inquiry_scan_interval < 2048)
    {
        max_rand = 127;
    }

    random_number = lc_generate_rand_number(max_rand);

    /**
     * Note: Consider EIR if there is a need to change this
     * minimum value.
     */
    if (random_number < 50)
    {
       random_number += 50;
    }

    status = (UCHAR) OS_START_TIMER(random_backoff_timer,
                                SLOT_VAL_TO_TIMER_VAL(random_number));
    if( status != BT_ERROR_OK )
    {
        return;
    }

    return;
}

#ifdef VER_1_1_SPECIFIC
/** 
 * Handles start of random backoff interrupt for inquiry scan procedure.
 * 
 * \param None.
 * 
 * \return None.
 */
void lc_handle_random_start_intr(void)
{
    UCHAR status;
    UINT16 random_number;
    UINT16 max_rand = MAX_RAND;
    
#ifdef ENABLE_LOGGER_LEVEL_2
	LC_LOG_INFO(LOG_LEVEL_LOW,RANDOM_START_INTR_ONLY_FOR_1_1_COMPATIBILITY,0,0);
#endif

#ifdef _DAPE_TEST_NEW_HW_INSTRUCTION_KILL_SCAN
    lc_kill_inquiry_scan_instruction();
#else	
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_INQUIRY);
#endif

    if(lmp_self_device_data.inquiry_scan_interval < 2048 )
    {
        max_rand = 127;
    }
    
    random_number = lc_generate_rand_number(max_rand);

    /* Start Random backoff timer. */
    status = (UCHAR) OS_START_TIMER(random_backoff_timer,
                SLOT_VAL_TO_TIMER_VAL(random_number));
    if( status != BT_ERROR_OK )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
		LC_LOG_ERROR(LOG_LEVEL_LOW,RANDOM_BACKOFF_TIMER_START_FAILED,0,0);
#endif

        return ;
    }

    random_backoff_status = TRUE;

    return;
}
#endif


/** 
 * Handles random backoff timeout interrupt from the baseband. 
 * 
 * \param None.
 * 
 * \return None.
 */
void lc_handle_random_backoff_timeout(TimerHandle_t timer_handle)
{
    lc_handle_start_inq_scan_state();    
    random_backoff_status = FALSE;
}

/** 
 * Programs baseband to enter inquiry scan state. This function is 
 * called when the page-scan and inquiry-scan-windows overlap, and 
 * inquiry-scan is preempted as page-scan has higher priority. This 
 * function is deprecated.
 * 
 * \param None.
 * 
 * \return None.
 */
void lc_handle_start_inq_scan_state(void)
{
    if ((lmp_self_device_data.scan_enable & 0x01) == 0 )
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LMP_LOG_INFO(LOG_LEVEL_HIGH, INQ_SCAN_IS_NOT_ENABLED_WHEN_RANDOM_BACKOFF_TIMER_FIRED, 0, 0);
#endif
        return;
    }

    /* Donot allow scan if already inquiry or page procedure is in progress */
    if ((lmp_self_device_data.lc_cur_dev_state & 
                (LC_CUR_STATE_PAGE | LC_CUR_STATE_INQUIRY)) != 0)
    {
        return;
    }

#ifdef _3DD_FUNCTION_SUPPORT_
#ifdef _SUPPORT_CSB_TRANSMITTER_
    if (!bt_3dd_var.is_csb_mode)
#endif
    {

        if (lmp_self_device_data.inquiry_scan_type == HCI_3DD_INQ_SCAN)
        {
            bt_3dd_driver_retrieve_inq_scan();
            return;
        }
    }
#endif

    if (lmp_feature_data.feat_page0[3] & ENHANCED_INQ_SCAN)
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER,
                                    BB_INQUIRY_SCAN_SECOND_ID);
    }
    else
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER,
                                    BB_INQUIRY_SCAN_FIRST_ID);
    }

    return;
}

/** 
* Retrieves inquiry-scan after being disabled by the Controller.
* 
* \param None.
* 
* \return None.
*/
void lc_retrieve_inq_scan(void)
{
    if (lmp_self_device_data.scan_enable & 0x01)
    {
        BB_write_baseband_register(INSTRUCTION_REGISTER, 
                                   BB_INQUIRY_SCAN_SECOND_ID);
    }

    return;
}

/** 
* Retrieves page-scan after being disabled by the Controller.
* 
* \param None.
* 
* \return None.
*/
void lc_retrieve_page_scan(void)
{
    UCHAR phy_piconet_id;
    UINT16 address;
    UINT16 con_reg_value;

#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_retrieve_page_scan_func != NULL)
    {
        if (rcp_lc_retrieve_page_scan_func(NULL))
        {
            return;
        }
    }
#endif
    DEF_CRITICAL_SECTION_STORAGE;

    if (lmp_self_device_data.scan_enable & 0x02)
    {
        MINT_OS_ENTER_CRITICAL();
        
        phy_piconet_id = lc_get_piconet_id_for_pagescan();

        if (phy_piconet_id > SCA_PICONET_MAX)
        {
            MINT_OS_EXIT_CRITICAL();
            RT_BT_LOG(RED, MSG_PAGE_SCAN_NO_PID, 0, 0);              
            return;       
        }  

        address = reg_PICONET_INFO[phy_piconet_id];
        
        con_reg_value = (UINT16) (phy_piconet_id << 11);
        
#ifdef _DAPE_TEST_FIX_NO_PAGE_SCAN
        UINT16 bb_reg;
        bb_reg = BB_read_baseband_register(PAGE_SCAN_WINDOW_REGISTER);
        if ((bb_reg>>14) != 0)
        {
            bb_reg &= (UINT16)(0x3FFF);
            BB_write_baseband_register(PAGE_SCAN_WINDOW_REGISTER, bb_reg);
        }
#endif

        BB_write_baseband_register(address, SCA_PICONET_SLAVE);
        BB_write_baseband_register(CONNECTOR_REGISTER, con_reg_value);
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_PAGE_SCAN);

        lc_pagescan_piconet_id = phy_piconet_id;

        MINT_OS_EXIT_CRITICAL();
    }

    return;
}


