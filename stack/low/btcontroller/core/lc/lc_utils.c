/***************************************************************************
Copyright (C) MindTree Ltd.
This module is a confidential and proprietary property of MindTree and
a possession or use of this module requires written permission of MindTree.
***************************************************************************/

/**
* \file
*  Contains the general utility functions used in the LC module.
*/

/********************************* Logger *************************/
enum { __FILE_NUM__= 44 };
/********************************* Logger *************************/


/* ========================= Include File Section ========================= */
#include "lc_internal.h"
#include "bt_fw_hci.h"
#include "lmp.h"
#include "lmp_internal.h"
#include "bt_fw_os.h"
#include "bt_fw_globals.h"
#include "vendor.h"
#include "lc.h"
#include "led_debug.h"
#include "mem.h"

#include "bt_fw_acl_q.h"
#include "lmp_pdu_q.h"

#include "lmp_1_2_defines.h"

#include "bt_fw_hci_2_1.h"

#include "bt_3dd.h"

/* ===================== Variable Declaration Section ===================== */

/* ================== Static Function Prototypes Section ================== */
UCHAR lc_get_no_of_piconets_connected(void);

#ifdef TEST_MODE
extern UINT16 lc_is_tx_test_mode;
extern UINT8 test_mode_sched_acl_pkt;
#endif

#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
extern UINT8 temp_lut_id[4];
#endif

/* Broadcast LMP FIFO free bytes. */
extern UINT16 bc_tx_fifo_free_bytes;
extern UINT16 programmable_rand;
#ifdef COMPILE_PARK_MODE
extern UCHAR lc_beacon_running_flag;
#endif
/* Indicate whether the scan timer in use for other purposes or not
* 0x0 => Not in use, 0x1 => in use */
UCHAR g_lc_scan_slot_timer_in_use = 0;

/* Indicate wether to pause scheduling new pkts when there is a MSS pending.
* 0 => Can schedule new pkts, 0x1=> do not schedule new pkts.
*/
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
UCHAR lc_pause_schedule_pkts = 0;
#endif
extern UINT8 lc_sco_pause_status;

/* ===================== Static Function Definition Section =============== */
LC_CUR_SCATTERNET_STATE lc_get_scatternet_state_from_phy_piconet_id(
    UCHAR phy_piconet_id);
void lc_reset_scatternet_state(void);

void lc_check_and_cleanup_bd_addr_regs_on_disc(UCHAR phy_piconet_id);

#ifdef COMPILE_ROLE_SWITCH
void lc_program_and_start_slot_counter(UINT32 duration);
#endif /* COMPILE_ROLE_SWITCH */

#ifdef _ROM_CODE_PATCHED_
PF_ROM_CODE_PATCH_FUNC rcp_lc_decide_packet_type_change = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_get_exp_transfer_time = NULL;
PF_ROM_CODE_PATCH_FUNC rcp_lc_check_and_enable_scans_in_scatternet_func = NULL;
#endif
/* ===================== Function Definition Section ====================== */
/**
* Frees all the LMP_PDUs currently queued in the lmp-connection-entity.
* This function also frees the PDU buffer and releases it from the OS pool.
*
* \param ce_index Index to the lmp-connection-entity
* \param all_flag If true then all lmp_pdu's will be freed.
*
* \return None.
*/
void lc_free_lmp_pdus(UINT16 ce_index, UCHAR all_flag)
{
    UCHAR am_addr;
    UCHAR piconet_id;

    am_addr = lmp_connection_entity[ce_index].am_addr;

    piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;

    pduq_clear_all_pdus_am_addr(am_addr, piconet_id, all_flag);

    return;
}


/**
* Resets the LC level variables. It also re-initializes all the
* LUT_EX_tables. It also resets the entries of the LC piconet scheduler,
* and its states.
*
* \param None.
*
* \return None.
*/
void lc_reset_global_variables(void)
{
    UCHAR count;
    DEF_CRITICAL_SECTION_STORAGE;

    lc_set_lc_cur_connecting_am_addr(INVALID_AM_ADDR);
    lc_set_lc_cur_device_state(LC_CUR_STATE_IDLE);
    lc_reset_scatternet_state();

    g_lc_scan_slot_timer_in_use = 0x0;
#ifndef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
    lc_pause_schedule_pkts = 0x0;
#endif
    random_backoff_status = FALSE;
    lc_current_interrupt_mask = 0xFFFF;

#ifndef _2801_BTON_DESIGN_
    // move after lps efuse setting
    LC_INIT_SLEEP_MODE_PARAM();
#endif

    /* Initialize LUT Structures */
    for(count = 0; count < LC_MAX_NUM_OF_LUT_EX_TABLES; count++)
    {
        lc_reset_lut_ex_table(count);
        
        lc_cont_crc_rx_cnt[count] = 0;
        lc_cont_crc_tx_cnt[count] = 0;
#ifdef COMPILE_SNIFF_MODE
        lc_sniff_cont_count[count] = 0;
#endif
        lc_is_tpoll_started[count] = 0;          
    }

    lc_rx_pkt_header_q.read_pointer = 0;
    lc_rx_pkt_header_q.write_pointer = 0;

    /* Initialize timers */
    if (periodic_inquiry_timer != NULL)
    {
        OS_DELETE_TIMER(& periodic_inquiry_timer);
    }

#ifdef COMPILE_PERIODIC_INQUIRY
    lc_periodic_inq_max_interval = 0;
    lc_periodic_inq_min_interval = 0;
#endif

#ifdef ENABLE_SCO
    lc_full_bandwidth_flag = FALSE;
    lc_acl_full_bandwidth_flag = FALSE;
#endif /* ENABLE_SCO */

#ifdef COMPILE_PARK_MODE
    lc_start_of_beacon = FALSE ;
    lmp_unpark_ce_index = INVALID_CE_INDEX;
    lc_beacon_running_flag = FALSE ;
#endif /* COMPILE_PARK_MODE */

    MINT_OS_ENTER_CRITICAL();
    for(count = 0; count < LMP_MAX_CE_DATABASE_ENTRIES; count++)
    {
        lmp_sup_timeout_var[count] = 0;
        lmp_sup_var_timeout_var[count] = 0;
    }
    MINT_OS_EXIT_CRITICAL();

#ifdef TEST_MODE
    lc_tci_pause_flag = FALSE;
    lc_test_mode_ack_recd = FALSE;

#ifdef _DUT_DELAYED_LOOPBACK_MODE_
    for (count = 0; count < 2; count++)
    {
        dut_mode_manager.buf[count].len = 0;
        dut_mode_manager.buf[count].lut_lower = 0;
        dut_mode_manager.buf[count].lut_type = 0;
        dut_mode_manager.buf[count].ppkt = NULL;
    }
    dut_mode_manager.work_space = 0;
#endif

#endif /* TEST_MODE */

#ifdef _IMPROVE_PCM_MUTE_CTRL_TIMING_
    g_sco_no_rx_count = 0;
    g_sco_no_rx_force_output_zero_en = FALSE;
#endif

    lc_sco_pause_status = 0x00;

#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
    for(count = 0; count < MAX_PICONET_CNT; count++)
    {
        temp_lut_id[count] = 0;
    }
#endif

    return;
}


/**
* Resets the lut-ex-table entries.
*
* \param lut_index Index to the lut-ex-table dtat-structure
*
* \return None.
*/
void lc_reset_lut_ex_table(UCHAR lut_index)
{
    LUT_EXTENSION_TABLE *lut_ptr;

    lut_ptr = &lut_ex_table[lut_index];

    /* Remote device seqn bit used to check duplicate
    packets from the remote device */
    lut_ptr->remote_seqn_bit = BB_REMOTE_INIT_SEQN_VAL;

    if ((lut_index >= LC_SCA_SLAVE_1_LUT) && (lut_index <= LC_SCA_SLAVE_4_LUT))
    {
        lut_index -= LC_SCA_SLAVE_1_LUT;
        lut_ptr->bc_seqn_bit = BB_REMOTE_INIT_SEQN_VAL;
        lut_ptr->lower_lut_address = reg_SCA_SLAVE_LOWER_LUT[lut_index];
        lut_ptr->upper_lut_address = reg_SCA_SLAVE_UPPER_LUT[lut_index];
    }
    else
    {
        lut_ptr->lower_lut_address = reg_MASTER_LOWER_LUT[lut_index];
        lut_ptr->upper_lut_address = reg_MASTER_UPPER_LUT[lut_index];
    }

    lut_ptr->index_in_CE = INVALID_CE_INDEX;
    lut_ptr->bb_flow = BB_GO;

    return;
}

/* At ptt = 1, we can choose following packet types:
        3-dh5 (0~1021), 2-dh5 (0~552), 
        3-dh3 (0~552), 2-dh3 (0~367), 
        3-dh1 (0~83), 2-dh1 (0~54), dm1 (0~17) 
 ** If we transfer N byte payload in the packet
 *** The duration of 3-dh1/3-dh3/3-dh5 = 138 + (N+4)/3 (us)
 *** The duration of 2-dh1/2-dh3/2-dh5 = 138 + (N+4)/2 (us)
 *** The duration of dm1 = 122 + (N+3)*3/2 (us)  */

UINT8 decide_rule_edr[7] = {
    LC_3_DH5_INDEX, LC_2_DH5_INDEX,
    LC_3_DH3_INDEX, LC_2_DH3_INDEX, 
    LC_3_DH1_INDEX, LC_2_DH1_INDEX, LC_DM1_INDEX};

/* At ptt = 0, we can choose following packet types:
        dh5 (0~339), dm5 (0~224), 
        dh3 (0~183), dm3 (0~121), 
        dh1 (0~27), dm1 (0~17) 
 ** If we transfer N byte payload in the packet
 *** The duration of dh1 (no FEC) = 125 + N (us)
 *** The duration of dh3/dh5 (no FEC) = 126 + N (us)
 *** The duration of dm1 (2/3 FEC) = 122 + (N+3)*3/2 (us)  
 *** The duration of dm3/dm5 (2/3 FEC) = 122 + (N+4)*3/2 (us) */

UINT8 decide_rule_basic[6] = {
    LC_DH5_INDEX, LC_DM5_INDEX,
    LC_DH3_INDEX, LC_DM3_INDEX, 
    LC_DH1_INDEX, LC_DM1_INDEX
};   

/**
* Decides the packet type based on the packets that are currently
* allowed, and the length of the packet. This function is called for
* every ACLU packet. (modify decision by austin)
*
* \param length Pointer to the length of the packet.
* \param pkt_type The type of the packet
* \param index Index to the lmp-connection-entity
*
* \return None.
*/
void lc_decide_packet_type(UINT16 *length,
                           UINT16 *pkt_type_lut,UINT16 index)
{
    UCHAR i;
    PKT_ALLOWED *pkts_allowed_ptr = NULL;    
    PKT_ALLOWED pkts_allowed_temp;

    pkts_allowed_ptr = &(lmp_connection_entity[index].pkts_allowed);
        
    if ((pkts_allowed_ptr->status) & (0x0001))
    {
        *pkt_type_lut = g_lc_pkt_type_lut[LC_DV_INDEX];
        if (*length > g_lc_pkt_type_max_len[LC_DV_INDEX])
        {
            *length = g_lc_pkt_type_max_len[LC_DV_INDEX];
        }
        return;
    }

    pkts_allowed_temp.status = pkts_allowed_ptr->status;

#ifdef _ROM_CODE_PATCHED_
    /* if it is required, we can modify allowed packet packet here for rate
       adaption - austin */
    if (rcp_lc_decide_packet_type_change != NULL)
    {
        /* analyze some packet status to modify it again via patched code */
        if (rcp_lc_decide_packet_type_change(&pkts_allowed_temp.status,
                                             length, index))
        {
            return;
        }
    }
#endif

#if defined(_BRUCE_FIX_CQDDR) && defined(COMPILE_CQDDR)
    if (!IS_USE_FOR_MUTE)
    {
        UCHAR remote_feature;
        remote_feature=lmp_connection_entity[index].feat_page0[3];        
        if(lmp_connection_entity[index].ptt_status == LMP_PTT_ENABLED)
        {
            if((remote_feature & BIT2) == FALSE)
            {
                pkts_allowed_temp.status &= ~(BIT(LC_3_DH1_INDEX) | BIT(LC_3_DH3_INDEX) | BIT(LC_3_DH5_INDEX));
                pkts_allowed_temp.status |= (BIT(LC_2_DH1_INDEX) | BIT(LC_2_DH3_INDEX) | BIT(LC_2_DH5_INDEX));      
            }
        }  
    }
#endif


    if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DM1_PACKET)
    {
        /* Select dm1 . */
        *pkt_type_lut = BB_DM1_LUT;                                    
    }
    else
    {
        UINT8 *decide_rule;
        UINT8 decide_len;
        
        if (lmp_connection_entity[index].ptt_status == LMP_PTT_ENABLED)
        {
            /* Support EDR rate packet type */   
            decide_len = 7;
            decide_rule = decide_rule_edr;         

            if (((*length <= LC_MAX_NUMBER_OF_BYTES_IN_2_DH1_PACKET) &&
                 (pkts_allowed_temp.status & (1 << LC_2_DH1_INDEX))) ||
                 ((*length <= LC_MAX_NUMBER_OF_BYTES_IN_3_DH1_PACKET) &&
                 (pkts_allowed_temp.status & (1 << LC_3_DH1_INDEX))))
            {
                /* check from one slot packet type */
                i = 4;
            }
            else if (((*length <= LC_MAX_NUMBER_OF_BYTES_IN_2_DH3_PACKET) &&
                     (pkts_allowed_temp.status & (1 << LC_2_DH3_INDEX))) ||
                     ((*length <= LC_MAX_NUMBER_OF_BYTES_IN_3_DH3_PACKET) &&
                     (pkts_allowed_temp.status & (1 << LC_3_DH3_INDEX))))
            {
                /* check from three slot packet type */                            
                i = 2; 
            }
            else
            {
                /* check from five slot packet type */                            
                i = 0;                       
            }            
        }
        else
        {
            /* Only support basic rate packet type */
            decide_len = 6;
            decide_rule = decide_rule_basic;

            if (((*length <= LC_MAX_NUMBER_OF_BYTES_IN_DH1_PACKET) &&
                 (pkts_allowed_temp.status & (1 << LC_DH1_INDEX))))
            {
                /* check from one slot packet type */
                i = 4;
            }
            else if (((*length <= LC_MAX_NUMBER_OF_BYTES_IN_DM3_PACKET) &&
                     (pkts_allowed_temp.status & (1 << LC_DM3_INDEX))) ||
                     ((*length <= LC_MAX_NUMBER_OF_BYTES_IN_DH3_PACKET) &&
                     (pkts_allowed_temp.status & (1 << LC_DH3_INDEX))))
            {
                /* check from three slot packet type */                            
                i = 2; 
            }
            else
            {
                /* check from five slot packet type */                            
                i = 0;                       
            } 
        }

        for (; i <= decide_len; i++)
        {
            if (pkts_allowed_temp.status & (1 << decide_rule[i]))
            {
                break;                          
            }
        }

#ifdef _PACKET_DECIDE_POLICY_NEW_       
        if (IS_PKT_DECIDE_2M_FIRST)
        {
            if (!(i & 0x01) && (decide_rule[i] != LC_DM1_INDEX))
            {
                /* when i is even */
                if ((pkts_allowed_temp.status & (1 << decide_rule[i + 1])) &&
                    (*length <= g_lc_pkt_type_max_len[decide_rule[i + 1]]))
                {                    
                    /* change 2M EDR packet */
                    i++;                        
                }
            }
        }
#endif



#ifdef _CCH_SC_ECDH_P256_TEST_PKT
    if ((*length > LC_MAX_NUMBER_OF_BYTES_IN_DM1_PACKET)&&
	(lmp_self_device_data.total_no_of_sco_conn == 0)&&
	(lmp_self_device_data.number_of_esco_connections == 0))
    {
        if(lmp_connection_entity[index].ptt_status != LMP_PTT_ENABLED)
        {
            if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DH1_PACKET)
            {
                *pkt_type_lut = BB_DH1_LUT;
            }
            else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DM3_PACKET)
            {
                *pkt_type_lut = BB_DM3_LUT;
            }
            else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DH3_PACKET)
            {
                *pkt_type_lut = BB_DH3_LUT;
            }
            else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DM5_PACKET)
            {
                *pkt_type_lut = BB_DM5_LUT;
            }
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_DH5_PACKET)
        {
            *pkt_type_lut = BB_DH5_LUT;
		}
		else
        {
            *pkt_type_lut = BB_DH5_LUT;
            *length = LC_MAX_NUMBER_OF_BYTES_IN_DH5_PACKET;
		}
    }else
    {
        if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_2_DH1_PACKET)
        {
            *pkt_type_lut = BB_2_DH1_LUT;
		}             
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_3_DH1_PACKET)
        {
            *pkt_type_lut = BB_3_DH1_LUT;
		}             
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_2_DH3_PACKET)
        {
            *pkt_type_lut = BB_2_DH3_LUT;
		}             
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_3_DH3_PACKET)
        {
            *pkt_type_lut = BB_3_DH3_LUT;
		}             
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_2_DH5_PACKET)
        {
            *pkt_type_lut = BB_2_DH5_LUT;
		}             
		else if (*length <= LC_MAX_NUMBER_OF_BYTES_IN_3_DH5_PACKET)
        {
            *pkt_type_lut = BB_3_DH5_LUT;
		}             
        else
        {
            *pkt_type_lut = BB_3_DH5_LUT;
            *length = LC_MAX_NUMBER_OF_BYTES_IN_3_DH5_PACKET;
		} 
    }
}else
{
        *pkt_type_lut = g_lc_pkt_type_lut[decide_rule[i]];                    

        if (*length > g_lc_pkt_type_max_len[decide_rule[i]])
        {
            *length = g_lc_pkt_type_max_len[decide_rule[i]];
        }
}
   
		

#else

        *pkt_type_lut = g_lc_pkt_type_lut[decide_rule[i]];                    

        if (*length > g_lc_pkt_type_max_len[decide_rule[i]])
        {
            *length = g_lc_pkt_type_max_len[decide_rule[i]];
        }              
#endif
		
    } 
}


/**
* Generates parity bits using the passed LAP.
*
* \param lap The lower address part
* \param parity_bits The data-structure where the generated parity
*                    bits are returned.
*
* \return None.
*/
void lc_generate_parity_bits(UINT32 lap, UCHAR *parity_bits)
{
    UINT32 barker_seq;
    UINT32 info_bits;
    UINT32 mod2;
    UINT32 mod1;
    UINT32 temp;
    UCHAR i;

    /* Barker sequence generation. */
    if(lap & 0x00800000)
    {
        barker_seq = (lap | 0x13000000);
    }
    else
    {
        barker_seq = lap | 0x2c000000;
    }
    
    info_bits = barker_seq ^ (LC_PN_SEQ_MSB >> 2);
    info_bits <<= 2;

    mod2 = info_bits;
    mod1 = 0x00;

    for(i = 0; i <= 29; i++)
    {
        if(mod2 & 0x80000000)
        {
            mod2 ^= LC_GEN_POLY_MSB;
            mod1 ^= LC_GEN_POLY_LSB;
        }
        
        mod2 = (mod2 << 1) | (mod1 >> 31);
        mod1 <<= 1;             
    }

    temp = ((mod2 & 0x3fffffff) << 2) | (mod1 >> 30);

    parity_bits[0]= (UCHAR)(temp);
    parity_bits[1] = (UCHAR)(temp >> 8);
    parity_bits[2] = (UCHAR)(temp >> 16);
    parity_bits[3] = (UCHAR)(temp >> 24);

    temp = (mod2) >> 30;
    
    parity_bits[4] = (UCHAR)(temp & 0x00000003);
    return;
}

void lc_generate_true_parity_bits(UINT32 lap, UCHAR *parity_bits)
{
    lc_generate_parity_bits(lap, parity_bits);
    parity_bits[0] ^= LC_PN_SEQ_LSB;
    parity_bits[1] ^= LC_PN_SEQ_LSB >> 8;
    parity_bits[2] ^= LC_PN_SEQ_LSB >> 16;
    parity_bits[3] ^= LC_PN_SEQ_LSB >> 24;
    parity_bits[4] ^= LC_PN_SEQ_MSB & 0x03;
}

UINT32 g_idum = 0x0L;
/**
* Generates a random number lesser than max-rand
*
* \param max_rand The generated random numner needs to be smaller than this.
*
* \return rand The random number generated.
*/
UINT16 lc_generate_rand_number(UINT16 max_rand)
{
    UINT16 rand;

    if(max_rand > programmable_rand)
    {
        max_rand = (UINT16) programmable_rand;
    }
    g_idum = (UINT32) ((1664525L * g_idum) + 1013904223L);
    rand = (g_idum >> 22) & 0xFFFF;
    rand %= (max_rand + 1);

    return rand;
}


/**
* Modifies particular baseband register. This function is used to reset
* particular bit(s) in a register. This function assumes that the
* register is RW.
*
* \param reg_offset The address of the register that is modified.
* \param val The value that needs to be written into the register.
*
* \return None.
*/
void AND_val_with_bb_reg(UINT16 reg_offset, UINT16 val)
{
    UINT16 read;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    read = BB_read_baseband_register(reg_offset);
    read = (UINT16) (read & val);
    BB_write_baseband_register(reg_offset, read);
    MINT_OS_EXIT_CRITICAL();

    return;
}


/**
* Modifies particular baseband register. This function is used to reset
* particular bit(s) in a register. This function assumes that the
* register is RW. This function will not have critical section. Se,
* this needs to be called ONLY from places where you are sure that
* this function is atomically executed, either from BB-interrupts or
* from already critical section.
*
* \param reg_offset The address of the register that is modified.
* \param val The value that needs to be written into the register.
*
* \return None.
*/
void AND_val_with_bb_reg_isr(UINT16 reg_offset, UINT16 val)
{
    UINT16 read;

    read = BB_read_baseband_register(reg_offset);
    read = (UINT16) (read & val);
    BB_write_baseband_register(reg_offset, read);

    return;
}


/**
* Modifies particular baseband register. This function is used to set
* particular bit(s) in a register. This function assumes that the
* register is RW.
*
* \param reg_offset The address of the register that is modified.
* \param val The value that needs to be written into the register.
*
* \return None.
*/
void OR_val_with_bb_reg(UINT16 reg_offset, UINT16 val)
{
    UINT16 read;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    read = BB_read_baseband_register(reg_offset);
    read = (UINT16) (read | val);
    BB_write_baseband_register(reg_offset, read);
    MINT_OS_EXIT_CRITICAL();

    return;
}


/**
* Modifies particular baseband register. This function is used to set
* particular bit(s) in a register. This function assumes that the
* register is RW. This function will not have critical section. Se,
* this needs to be called ONLY from places where you are sure that
* this function is atomically executed, either from BB-interrupts or
* from already critical section.
*
* \param reg_offset The address of the register that is modified.
* \param val The value that needs to be written into the register.
*
* \return None.
*/
void OR_val_with_bb_reg_isr(UINT16 reg_offset, UINT16 val)
{
    UINT16 read;

    read = BB_read_baseband_register(reg_offset);
    read = (UINT16) (read | val);
    BB_write_baseband_register(reg_offset, read);

    return;
}

void Update_val_with_bb_reg(UINT16 offset, UINT16 val, UINT16 mask)
{
    UINT16 read;
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    read = BB_read_baseband_register(offset);
    read &= ~mask;    
    read |= val & mask;
    BB_write_baseband_register(offset, read);
    MINT_OS_EXIT_CRITICAL();
}

void Update_val_with_bb_reg_isr(UINT16 offset, UINT16 val, UINT16 mask)
{
    UINT16 read;

    read = BB_read_baseband_register(offset);
    read &= ~mask;    
    read |= val & mask;
    BB_write_baseband_register(offset, read);
}

#ifdef POWER_CONTROL

/**
* Checks if the radio power level can be decreased from current level.
*
* \param power_level the current level of the radio tx power.
*
* \return TRUE if the radio is not already at min power
*         FALSE otherwise
*/
UCHAR lc_check_for_decr_power_level(UCHAR power_level)
{
    UCHAR flag = FALSE;

    if (power_level > MIN_RADIO_TX_POWER)
    {
        flag = TRUE;
    }

    return flag;
}


/**
* Checks if the radio power level can be increased from current level.
*
* \param power_level the current level of the radio tx power.
*
* \return TRUE if the radio is not already at max power
*         FALSE otherwise
*/
UCHAR lc_check_for_incr_power_level(UCHAR power_level)
{
    UCHAR flag = FALSE;

    if(power_level < MAX_RADIO_TX_POWER)
    {
        flag = TRUE;
    }

    return flag;
}


/**
* Increases the power-level of the radio by one unit, after checking
* that the radio is not already at max power.
*
* \param power_level the current level of the radio tx power.
*
* \return power_level the updated power(if any).
*/
UCHAR lc_incr_power_level(UCHAR power_level)
{
    if(power_level < MAX_RADIO_TX_POWER)
    {
        power_level++;
    }

    return power_level;
}


/**
* Decreases the power-level of the radio by one unit, after checking
* that the radio is not already at min power.
*
* \param power_level the current level of the radio tx power.
*
* \return power_level the updated power(if any).
*/
UCHAR lc_decr_power_level(UCHAR power_level)
{
    if(power_level > MIN_RADIO_TX_POWER)
    {
        power_level--;
    }

    return power_level;
}
#endif


#ifdef COMPILE_PARK_MODE
/**
* Programs beacon parameters to the baseband. The function spins for a
* clk1 transition and then programs the parameters to the baseband.
*
* \param ce_index Index to the lmp-connection-entity
*
* \return None.
*/
void lc_program_beacon(UINT16 ce_index)
{
    UINT16 offset = 0;
    UINT32 master_clock;
    UINT32 new_master_clock;
    UINT16 Dbeacon;
    UINT16 delta_offset;
    UINT16 NewDbeacon;
    UINT16 address ;
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR m_access;
    DEF_CRITICAL_SECTION_STORAGE;

    UINT16 reg2;
    UINT16 reg1;

    ce_ptr = &lmp_connection_entity[ce_index];

    if (ce_ptr->remote_dev_role == SLAVE)
    {
#ifdef ENABLE_LOGGER_LEVEL_2
        LC_LOG_INFO(LOG_LEVEL_LOW,
                    "Writing broadcast packet type to am_addr zero.");
#endif

        /* Load default NULL packet into broadcast LUT. */
        address = MASTER_BROADCAST_LOWER_LUT_ADDR;
        BB_write_baseband_register((UINT16) address,
                                    LC_SLAVE_DEFAULT_PACKET_TYPE);
    }

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_KILL_BEACON);
    BB_write_baseband_register(T_B_REGISTER, ce_ptr->Tbeacon);
    BB_write_baseband_register_upper_octet(
        BEACON_PACKET_REPETETION_NO_REGISTER, ce_ptr->Nbeacon);

    BB_write_baseband_register_lower_octet(
        RE_TRANSMISSION_COUNT_REGISTER, ce_ptr->Nbeacon);

    BB_write_baseband_register_upper_octet(
        DELTA_B_REGISTER, ce_ptr->Delta_beacon);

    m_access = (UCHAR)BB_read_baseband_register_upper_octet(M_ACCESS_REGISTER);
    m_access = (UCHAR) (m_access & 0xF0);
    m_access = (UCHAR) (m_access | ce_ptr->M_access);
    BB_write_baseband_register_upper_octet(M_ACCESS_REGISTER, m_access);

    BB_write_baseband_register_lower_octet(T_ACCESS_REGISTER, ce_ptr->T_access);

    BB_write_baseband_register_lower_octet(D_ACCESS_REGISTER, ce_ptr->D_access);

    BB_write_baseband_register_lower_octet(N_POLL_REGISTER, ce_ptr->N_poll);

    ce_ptr->N_acc_slots += 1;
    ce_ptr->N_acc_slots &= (~0x1);

    BB_write_baseband_register_upper_octet(N_ACC_SLOTS_REGISTER,
                                           ce_ptr->N_acc_slots);

    /* Calculate modified Dsco from Dsco, TSco and master clock */
    if(ce_ptr->remote_dev_role == SLAVE)
    {
        /* Read native clock. */
        reg2 = NATIVE_CLOCK2_REGISTER;
        reg1 = NATIVE_CLOCK1_REGISTER;
    }
    else
    {
        /* Use p1 registers. */
        reg2 = PICONET_CLOCK2_REGISTER;
        reg1 = PICONET_CLOCK1_REGISTER;

        if(ce_ptr->phy_piconet_id == SCA_PICONET_SECOND)
        {
            /* Use p2 registers. */
            reg2 = PICONET2_CLOCK2_REGISTER;
            reg1 = PICONET2_CLOCK1_REGISTER;
        }
        else if(ce_ptr->phy_piconet_id == SCA_PICONET_THIRD)
        {
            /* Use p3 registers. */
            reg2 = PICONET3_CLOCK2_REGISTER;
            reg1 = PICONET3_CLOCK1_REGISTER;
        }
        else if(ce_ptr->phy_piconet_id == SCA_PICONET_FOURTH)
        {
            /* Use p4 registers. */
            reg2 = PICONET4_CLOCK2_REGISTER;
            reg1 = PICONET4_CLOCK1_REGISTER;
        }
    }

    master_clock = BB_read_baseband_register(reg2);
    master_clock <<= 16;
    master_clock |= BB_read_baseband_register(reg1);

    /* Shift clock by one bit to ignore clk0 bit */
    master_clock >>= 1;
    /* Ignore 27th Bit also - this is for Initialization 2 */
    master_clock &= BT_CLOCK_26_BITS;

    new_master_clock = 0;

    while( (master_clock + 1) >= new_master_clock)
    {
        new_master_clock = BB_read_baseband_register(reg2);
        new_master_clock <<= 16;
        new_master_clock |= BB_read_baseband_register(reg1);

        /* Shift clock by one bit to ignore clk0 bit */
        new_master_clock >>= 1;
        /* Ignore 27th Bit also - this is for Initialization 2 */
        new_master_clock &= BT_CLOCK_26_BITS;

        if(new_master_clock == 0)
        {
            break;
        }
    }

    master_clock = new_master_clock;

    if((master_clock != 0) && (ce_ptr->Tbeacon  != 0))
    {
        offset = (UINT16) (master_clock % ce_ptr->Tbeacon);
    }

    Dbeacon = ce_ptr->Dbeacon ;

    delta_offset = (UINT16) (ce_ptr->Tbeacon - offset);
    NewDbeacon = (UINT16) (delta_offset + Dbeacon);
    if(NewDbeacon > ce_ptr->Tbeacon)
    {
        NewDbeacon = (UINT16) (NewDbeacon - ce_ptr->Tbeacon);
    }

    if(NewDbeacon < LC_MIN_DBEACON_VALUE)
    {
        NewDbeacon += ce_ptr->Tbeacon;
    }

    /* Configure all required registers for starting beacon */
    BB_write_baseband_register(D_B_REGISTER, NewDbeacon);

    UINT16 lcl_addr = 0;
    BZ_REG_S_PICONET_INFO lcl_val;

    {
        if (ce_ptr->phy_piconet_id <= SCA_PICONET_MAX)
        {
            lcl_addr = reg_PICONET_INFO[ce_ptr->phy_piconet_id];
        }
        else
        {
            lcl_addr = PICONET2_INFO_REGISTER;
            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, ce_ptr->phy_piconet_id);      
        }

        *(UINT16*)&lcl_val = BB_read_baseband_register(lcl_addr);

        if(ce_ptr->remote_dev_role == SLAVE)
        {
            /* We are master. */
            lcl_val.park_config = 2;
        }
        else
        {
            /* We are slave. */
            lcl_val.park_config = 1;
        }

        BB_write_baseband_register(lcl_addr, *(UINT16*)&lcl_val);
    }

    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_START_BEACON);

    if(ce_ptr->remote_dev_role == MASTER)
    {
        slave_ce_index = ce_index;
        lc_program_tolerance(NewDbeacon, ce_index);
    }
    else
    {
        BB_modify_xtol_in_scatternet(0x0, ce_ptr->phy_piconet_id);
    }

    MINT_OS_EXIT_CRITICAL();

    RT_BT_LOG(LOG_LEVEL_HIGH, PARK_MSG_PG_BEACON, 6, 
            lcl_addr, *(UINT16*)&lcl_val, ce_index, ce_ptr->phy_piconet_id, 
            ce_ptr->remote_dev_role, NewDbeacon);
    return;
}
#endif /* COMPILE_PARK_MODE */

/**
* Selects the am-addr and the packet to be scheduled next. It also
* checks the number of entries in the scheduler, and if fills all
* the free entries in the scheduler.
* (This function shall not be called in ISR)
*
* \param piconet_id The phy piconet ID for which the scheduler has to be invoked.
*
* \return None.
*/
void  lc_invoke_scheduler(UCHAR phy_piconet_id)
{
    UINT16 ce_index = INVALID_CE_INDEX;
    LC_SCHEDULED_PKT *schd;
    LC_PICONET_SCHEDULER *piconet_schd;
    UINT16 frag_ptr;
    UINT16 pkt_len;
    UINT16 pkt_type_lut;
    UINT16 allowed_len;
    UINT8 pkt_src;
    UCHAR bc_flag = 0;
    UCHAR selected_am_addr; /* my am_addr */
    UCHAR lut_index = 0;
    HCI_ACL_DATA_PKT *acl_pkt = NULL;
    LMP_PDU_PKT *lmp_pkt = NULL;
    INT8 i;
    INT8 n;
    UCHAR queued_pkt = SCHEDULED_PKT_TYPE_NONE;
    UCHAR reschedule_flag = RESCHEDULE_FLAG_NONE;
    UINT16 reschedule_ce_index = 0;
    LMP_CONNECTION_ENTITY *ce_ptr;
    
	DEF_CRITICAL_SECTION_STORAGE;

    do
    {
        if (lmp_self_device_data.lc_no_of_connections[phy_piconet_id] == 0)
        {
            break;  
        }
#ifdef _ENABLE_COMPILE_NESTED_PAUSE_RESUME_
        if ((lmp_mss_state != LMP_MSS_INIT))
        {
            RT_BT_LOG(GRAY, LC_UTILS_1668, 1, lmp_mss_state);
            break;
        }
#else
        if ((lmp_mss_state != LMP_MSS_INIT) || (lc_pause_schedule_pkts == 0x1))
        {
            RT_BT_LOG(GRAY, LC_UTILS_1668, 1, lmp_mss_state);
            break;
        }      
#endif
#if defined(TEST_MODE)
        /* do not run sw scheduler when we schedule acl data pkt
           in test mode - austn */
        if ((phy_piconet_id == 0) && (test_mode_sched_acl_pkt == TRUE))
        {
            break;  
        }
#endif

        piconet_schd =  &lc_piconet_scheduler[phy_piconet_id];

        if (piconet_schd->lc_allowed_pkt_cnt == 0)
        {
            /* This should never happen. */
            break;  
        }

        if (piconet_schd->donot_schedule_pkt != 0x0)
        {
            /* Waiting for disconnect complete */
            /* Do not schedule pkt, still the donot_schedule_pkt == 0x0 */
            break;
        }

        if (lmp_self_device_data.number_of_hlc == 0)
        {
            /* check any tx pkt is ready for sw schedular */
            for (i = 0; i < LC_MAX_SCH_INFO; i++)
            {
                if (piconet_schd->lc_scheduled_pkt_info[i].tx_status != LC_TX_IDLE)
                {
                    break;
                }
            }

            /* if no tx pkt is ready, we can reset the pkt fifo of sw schedular */
            if (i == LC_MAX_SCH_INFO)
            {
                lc_piconet_scheduler[0].lc_allowed_pkt_cnt = LC_MAX_SCH_INFO;
                lc_piconet_scheduler[0].rptr = 0;
                lc_piconet_scheduler[0].wptr = 0;
            }
        }

        n = piconet_schd->lc_allowed_pkt_cnt;
        for (i = 0; i < n; i++)
        {
            pkt_src = LC_NULL;

            queued_pkt = SCHEDULED_PKT_TYPE_NONE;

            /*--------------------------------------*/
            /* check we need to reschedule any pkt  */
            /*--------------------------------------*/        
            if (reschedule_flag == RESCHEDULE_FLAG_NONE)
            {
                /* do not need to re-schedule any now. 
                   so we get next pdu from list */               
                lmp_pkt = pduq_get_next_pdu(&selected_am_addr, phy_piconet_id);
            }

            /* check any pending lmp pkt for sent or not */

            if (lmp_pkt != (LMP_PDU_PKT *)NULL)
            {
                pkt_len = lmp_pkt->pdu_length;

                if (selected_am_addr != BC_AM_ADDR)
                {
                    /* it is an unicast PDU. */

                    lut_index = lc_get_lut_index_from_phy_piconet_id(
                                    selected_am_addr, phy_piconet_id);

                    ce_index = lut_ex_table[lut_index].index_in_CE;

                    if (ce_index == INVALID_CE_INDEX)
                    {
                        break;
                    }

                    ce_ptr = &lmp_connection_entity[ce_index];

                    /* need to rescheduler any pkt ? */

                    if (ce_ptr->aclq_resch_flag != RESCHEDULE_FLAG_NONE)
                    {
                        /* if yes, reset the status of lmp pkt in the list */
                        pduq_reset_pdu_wo_failed(lmp_pkt);                        
                        lmp_pkt = (LMP_PDU_PKT *)NULL;
                        
                        /* To retry with acl alone */
                        reschedule_flag = ce_ptr->aclq_resch_flag;
                        reschedule_ce_index = ce_index;

                        i = i - 1;
                        continue;
                    }

                    /* decide packet type and bc flag for LMP pdu */
                    if (((ce_ptr->pkts_allowed.status & (1 << LC_DV_INDEX)) == TRUE) &&
                        (lmp_pkt->pdu_length <= LMP_MAX_NUMBER_OF_BYTES_IN_DV_PACKET) &&
                        (lmp_pkt->use_dm1 == FALSE))
                    {
                        pkt_type_lut = BB_DV_LUT;
                    }
                    else
                    {
                        pkt_type_lut = BB_DM1_LUT;
                    }

                    bc_flag = LC_DATA_ACTIVE_FLAG;
                }
                else
                {
                    /* This is a broadcast PDU. */
                    pkt_type_lut = BB_DM1_LUT;
                    bc_flag = LC_ACTIVE_BC_FLAG;
                } /* end of if (selected_am_addr != BC_AM_ADDR) */

                pkt_src = LC_PDU;
                queued_pkt = SCHEDULED_PKT_TYPE_LMP_PDU;
            } /* end of if (lmp_pkt != (LMP_PDU_PKT *)NULL) */

            /*
            * Force queue zero length pkt -
            * lmp packet is waiting.
            */
            if (reschedule_flag == RESCHEDULE_FLAG_ZERO_LENGTH)
            {
                LC_LOG_INFO(LOG_LEVEL_LOW,ZERO_LENGTH_PACKET_QUEUED,0,0);

                ce_index = reschedule_ce_index;
                ce_ptr = &lmp_connection_entity[ce_index];                
                selected_am_addr = ce_ptr->am_addr;

                /* and Queue zero length pkt */
                acl_pkt = aclq_get_zero_length_pkt(phy_piconet_id);
                pkt_len = 0;
                frag_ptr = 0;
                pkt_type_lut = BB_DM1_LUT;
                ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_NONE;
                ce_ptr->is_last_sent_zero_l_l2cap = 0x1;
                bc_flag = (UCHAR)(acl_pkt->broadcast_flag);
                pkt_src = LC_L2CAP;
                queued_pkt = SCHEDULED_PKT_TYPE_ACLU_DATA;
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
                RT_BT_LOG(WHITE, DAPE_TEST_LOG556, 0, 0);
#endif
            }

            /* check any pending acl pkt for sent or not */

            if (queued_pkt == SCHEDULED_PKT_TYPE_NONE)
            {
                /* get next acl pkt */
                acl_pkt = aclq_get_next_acl_pkt(&selected_am_addr, &pkt_len,
                                    &frag_ptr, &pkt_type_lut, phy_piconet_id);

                if (acl_pkt != (HCI_ACL_DATA_PKT *)NULL)
                {
                    LMP_GET_CE_INDEX_FROM_CONN_HANDLE(
                         (UINT16)acl_pkt->connection_handle, &ce_index);
                    
                     if (ce_index != INVALID_CE_INDEX)
                     {
                         ce_ptr = &lmp_connection_entity[ce_index];
                    
                         if (ce_ptr->aclq_resch_flag == RESCHEDULE_FLAG_ZERO_LENGTH)
                         {
                             LC_LOG_INFO(LOG_LEVEL_LOW,ZERO_LENGTH_PACKET_QUEUED,0,0);
                             
                             /* Give back this fragment */
                    
                             aclq_reset_acl_pkt_wo_failed(acl_pkt,
                                   ce_ptr->phy_piconet_id);
                    
                             /* and Queue zero length pkt */
                             acl_pkt = aclq_get_zero_length_pkt(phy_piconet_id);
                             pkt_len = 0;
                             frag_ptr = 0;
                             pkt_type_lut = BB_DM1_LUT;
                         }
                         ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_NONE;
                    
                         if (aclq_is_zero_length_pkt(acl_pkt, 
                                                    phy_piconet_id) == TRUE)
                         {
                             ce_ptr->is_last_sent_zero_l_l2cap = 0x1;
                         }
                         else
                         {
                             ce_ptr->is_last_sent_zero_l_l2cap = 0x0;
                         }
                     } /* end of if(ce_index != INVALID_CE_INDEX) */

                     queued_pkt = SCHEDULED_PKT_TYPE_ACLU_DATA;
                     bc_flag = (UCHAR)(acl_pkt->broadcast_flag);
                     pkt_src = LC_L2CAP;
                }
            } /* end of if(queued_pkt == 0) */

            if (queued_pkt == SCHEDULED_PKT_TYPE_NONE)
            {        
#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
                UINT8 aclq_resch_flag_is_zero_length=0;
                ce_index = lut_ex_table[temp_lut_id[phy_piconet_id]].index_in_CE;              
                if (ce_index == INVALID_CE_INDEX)
                {               
                    break;
                }         
                LMP_CONNECTION_ENTITY *ce_ptr;                
                ce_ptr = &lmp_connection_entity[ce_index];
                if(ce_ptr->aclq_resch_flag == RESCHEDULE_FLAG_ZERO_LENGTH)
                {
                    LC_LOG_INFO(LOG_LEVEL_LOW,ZERO_LENGTH_PACKET_QUEUED,0,0);
                    selected_am_addr = ce_ptr->am_addr;
                    /* and Queue zero length pkt */
                    acl_pkt = aclq_get_zero_length_pkt(phy_piconet_id);
                    pkt_len = 0;
                    frag_ptr = 0;
                    pkt_type_lut = BB_DM1_LUT;
                    ce_ptr->aclq_resch_flag = RESCHEDULE_FLAG_NONE;
                    ce_ptr->is_last_sent_zero_l_l2cap = 0x1;
                    bc_flag = (UCHAR)(acl_pkt->broadcast_flag);
                    pkt_src = LC_L2CAP;
                    queued_pkt = SCHEDULED_PKT_TYPE_ACLU_DATA;
#ifdef _DAPE_TEST_SEND_ZERO_LEN_PKT_BY_VENDOR_CMD
                    RT_BT_LOG(WHITE, DAPE_TEST_LOG556, 0, 0);
#endif                  
                    aclq_resch_flag_is_zero_length= 1;   
                }
       
                if(aclq_resch_flag_is_zero_length==0)
                {
                    break;
                }
#endif            
            }

            LC_EXIT_SM_MODE();

            allowed_len = pkt_len;

            MINT_OS_ENTER_CRITICAL();  

            /* get the pointer of pkt infomation of scheduler */
    		schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->wptr];   
            if (piconet_schd->lc_allowed_pkt_cnt != 0)
            {
                piconet_schd->lc_allowed_pkt_cnt--;
            }
            
            schd->selected_am_addr = selected_am_addr;
            schd->pkt_src = pkt_src;
            schd->pkt_type_lut = pkt_type_lut;

            if (queued_pkt == SCHEDULED_PKT_TYPE_LMP_PDU)
            {
                schd->lch = BB_LMP_PKT;
                schd->packet_ptr = (void*) lmp_pkt;
                frag_ptr = 0;

                /* we can use dma directly and don't copy memory once again */                
                if (lmp_pkt->pdu_length != 0)
                {        
                    schd->txdesc[0].DWord0 = 0;
                    schd->txdesc[0].DWord1 = 0;
                    schd->txdesc[0].start_addr = (UINT32)lmp_pkt->payload_content;
                    schd->txdesc[0].len = lmp_pkt->pdu_length;
                    schd->txdesc[0].isLast = TRUE;                    
                    schd->txdesc_used_cnt = 1;
                }
                else
                {
                    schd->txdesc_used_cnt = 0;                    
                }
            }
            else if (queued_pkt == SCHEDULED_PKT_TYPE_ACLU_DATA)
            {
                if (frag_ptr == 0)
                {
                    /* First fragment */
                    if (acl_pkt->packet_boundary_flag != L_CH_L2CAP_CONT)
                    {
                        schd->lch = BB_L2CAP_START_PKT;
                    }
                    else
                    {
                        schd->lch = BB_L2CAP_CONTI_PKT;
                    }
                }
                else
                {
                    /* Next fragement l_ch */
                    schd->lch  = BB_L2CAP_CONTI_PKT;
                }
                schd->packet_ptr = (void*) acl_pkt;

                aclq_write_to_baseband_TX_FIFO(acl_pkt, allowed_len,
                                schd->selected_am_addr, phy_piconet_id);
            } /* end of else if(queued_pkt == SCHEDULED_PKT_TYPE_ACLU_DATA) */

            schd->frag_len = allowed_len;
            schd->ce_index = ce_index;
            schd->bc_flag = bc_flag;
            schd->tx_status = LC_TX_READY;
            schd->wait_count = 0;

#if 0
            RT_TMP_LOG("(sche: len %d q %d ch %d w %d r %d allow %d)\n", allowed_len, queued_pkt, schd->lch,
                                                        piconet_schd->wptr, piconet_schd->rptr,
                                                        piconet_schd->lc_allowed_pkt_cnt);
#endif
            
            piconet_schd->wptr++;            
            MINT_OS_EXIT_CRITICAL();  
#ifdef _BRUCE_FIX_DROP_ZERO_LEN_DM1_WHEN_KEEP_RETRY
            /*
            LMP_CONNECTION_ENTITY *ce_ptr;
            ce_ptr = &lmp_connection_entity[schd->ce_index];        
            LC_SCHEDULED_PKT *schd1;
            schd1 = &piconet_schd->lc_scheduled_pkt_info[!piconet_schd->rptr];
            RT_BT_LOG(GRAY,BRUCE_DBG_9,9,piconet_schd->rptr,piconet_schd->wptr,
                schd->tx_status,schd1->tx_status,schd->ce_index,schd1->ce_index,ce_ptr->aclq_resch_flag,
                piconet_schd->lc_allowed_pkt_cnt,
                22);*/
#endif

            /* Reset reschedule_flag */
            reschedule_flag = RESCHEDULE_FLAG_NONE;            
        } /* end of for (i = 0; i < n; i++) */    
    }
    while (0);     
}

/**
* Extracts the lut-index from the ce-index.
*
* \param ce_index Index to the lmp-connection-entity
*
* \return lut-index, Index to the lut-ex-table data structure.
*/
INLINE UCHAR lc_get_lut_index_from_ce_index(UINT16 ce_index)
{
    UCHAR lut_index;

    lut_index = lc_get_lut_index_from_phy_piconet_id(
                lmp_connection_entity[ce_index].am_addr,
                lmp_connection_entity[ce_index].phy_piconet_id);
#ifdef _DAPE_TEST_CHK_CSB_SYNC_TX
#if 0
RT_BT_LOG(GREEN, YL_DBG_HEX_3, 3,
ce_index,
lmp_connection_entity[ce_index].am_addr,
                lmp_connection_entity[ce_index].phy_piconet_id);
#endif
#endif
    return lut_index;
}

/**
* Initiates the SEQN in scatternet. Make sure that you call this
* function from interrupts or from critical section. So, we dont
* need ENTER_CRITICAL and EXIT_CRITICAL in this function.
*
* \param am_addr The am_addr of the connection.
* \param ex_lut Pointer to the lut_ex_table of the connection.
* \param piconet_id Phy piconet ID of the connection.
*
* \return None.
*/
void lc_init_seqn_scatternet(UCHAR am_addr,
                             LUT_EXTENSION_TABLE  *ex_lut, UCHAR piconet_id)
{
    UINT16 temp_reg;

    temp_reg = (UINT16)( (am_addr << 5) | (piconet_id << 11) );

    BB_write_baseband_register(CONNECTOR_REGISTER, temp_reg);

    /* Initialize seqn bit variables. */
    ex_lut->remote_seqn_bit = BB_REMOTE_INIT_SEQN_VAL;

    if (lc_sca_get_piconet_role(piconet_id) == SLAVE)
    {
        ex_lut->bc_seqn_bit = BB_REMOTE_INIT_SEQN_VAL;
    }

    /* Make seqn and arqn bits 0 in the baseband lut. */
    temp_reg = BB_read_baseband_register(ex_lut->upper_lut_address);
    temp_reg &= 0xfff8;
    temp_reg |= 0x0001;
    BB_write_baseband_register(ex_lut->upper_lut_address,temp_reg);

    /* Issue opcode to initilize seqn in the baseband. */
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_SEQN_INIT);

    return;
}

#ifdef COMPILE_RSSI_REPORTING
/**
* Calculates and stores RSSI value in the lmp-connection-entity
* based on the formula provided by the radio spec.
*
* \param rssi The RSSI averaged value read from the baseband.
* \param ce_index Index to lmp-connection-entity.
*
* \return None.
*/
void lc_calculate_and_store_rssi(UINT16 rssi, UINT16 ce_index)
{
    if (ce_index != INVALID_CE_INDEX)
    {
        LMP_CONNECTION_ENTITY *ce_ptr;
        
        ce_ptr = &lmp_connection_entity[ce_index];
        
#ifdef _YL_MODEM_RSSI_MAPPING
        // TODO: why check RSSI = 0 or NOT ??
#endif
        if (rssi != 0)
        {
            /* Increment the number of rssi samples here. */
            ce_ptr->rssi_samples_count++; 
        
            ce_ptr->rssi = rssi;
            ce_ptr->rssi_value_accumulate += rssi;
        }

#ifdef ENABLE_LOGGER_LEVEL_2            
        LMP_LOG_INFO(LOG_LEVEL_HIGH, ZERO_RSSI, 3, rssi, ce_ptr->rssi_samples_count , ce_ptr->rssi_value_accumulate);
#endif
    }

    return;
}


/**
* Calculates and RSSI value by calling Bluewiz log function.
*
* \param rssi The RSSI averaged value read from the baseband.
*
* \return rssi_log The calculated RSSI value.
*/
INT8 lc_calculate_log_from_rssi(UINT16 rssi)
{
    INT8 rssi_dbm;

#ifdef _YL_MODEM_RSSI_MAPPING
    rssi_dbm = (rssi << 1) + otp_str_data.efuse_modem_rssi0_pin_dBm;
    if(rssi_dbm < otp_str_data.efuse_modem_rssi0_pin_dBm)
    {
        rssi_dbm = otp_str_data.efuse_modem_rssi0_pin_dBm;
    }
#else
    rssi_dbm = (rssi << 1) - 90;
    if(rssi_dbm < -90)
    {
        rssi_dbm = -90;
    }
#endif

    if(rssi_dbm > 0)
    {
        rssi_dbm = 0;
    }

    return rssi_dbm;
}
#endif /* COMPILE_RSSI_REPORTING */

/**
* Calculates the transfer time to set the TRX timing for BT
* in co-exsitence application
*
* \param conn_handle the connection_handle.
*
* \return expected transfer time.
*/
UINT16 lc_get_expect_transfer_time(UINT16 conn_handle, 
                               UINT8  retry_index,
                               UINT8  pkt_type)
{
    UINT16 slot_pair_number = 0;
    UINT16 ce_index;
    UINT8 piconet_id;
    UINT8 i; 
    LC_SCHEDULED_PKT *ppkt_info;
    UINT8 cur_rptr;
    UINT32 lcl_rx_pkt_type;
    UINT16 pkt_cnt;
    UINT32 total_len;
    UINT8 am_addr;
    
    if (LMP_GET_CE_INDEX_FROM_CONN_HANDLE(conn_handle, &ce_index) !=  API_SUCCESS)
    {
        return 0;
    }

    if (lmp_ch_to_ce_index_table[conn_handle - 1].status == DEDICATED)
    {
#ifdef _ROM_CODE_PATCHED_
        if (rcp_lc_get_exp_transfer_time != NULL)
        {
            UINT16 result;            
            if (rcp_lc_get_exp_transfer_time(&result, 
                                         &lmp_connection_entity[ce_index]))
            {
                return result;
            }
        }
#endif

        piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;
        ppkt_info = lc_piconet_scheduler[piconet_id].lc_scheduled_pkt_info;
        cur_rptr  = lc_piconet_scheduler[piconet_id].rptr;
        am_addr = lmp_connection_entity[ce_index].am_addr;
            
        /* check any ready packet in sw schedule */
        for (i = 0; i < LC_MAX_SCH_INFO; i++)
        {
            if ((ppkt_info[cur_rptr].tx_status != LC_TX_IDLE) &&
                (ppkt_info[cur_rptr].ce_index == ce_index))
            {
                lcl_rx_pkt_type = 1 << (ppkt_info[cur_rptr].pkt_type_lut >> 12);

                if (lmp_connection_entity[ce_index].ptt_status == LMP_PTT_ENABLED)
                {
                    /* check EDR packet */
                    if (lcl_rx_pkt_type & BB_RX_ISR_PTT1_ONE_SLOT_ACL_PKT)
                    {
                        slot_pair_number++;                        
                    }
                    else if (lcl_rx_pkt_type & BB_RX_ISR_PTT1_THREE_SLOT_ACL_PKT)
                    {
                        slot_pair_number += 2;                          
                    }
                    else
                    {
                        slot_pair_number += 3;                          
                    }
                }
                else
                {
                    /* check BR packet */                    
                    if (lcl_rx_pkt_type & BB_RX_ISR_PTT0_ONE_SLOT_ACL_PKT)
                    {
                        slot_pair_number++;   
                    }
                    else if (lcl_rx_pkt_type & BB_RX_ISR_PTT0_THREE_SLOT_ACL_PKT)
                    {
                        slot_pair_number += 2;                          
                    }
                    else
                    {
                        slot_pair_number += 3;                          
                    }
                }
            }
            cur_rptr = (cur_rptr + 1) & (LC_MAX_SCH_INFO - 1);
        }       
        
        /* check any pending lmp packet in the list */        
        pkt_cnt = pduq_get_no_of_pdus_am_addr(am_addr, piconet_id, TRUE);
        slot_pair_number += pkt_cnt;
        
        /* check any pending acl packet in the list */
        pkt_cnt = aclq_get_no_of_pkts_am_addr(am_addr, piconet_id, 
                                              &total_len, TRUE);
        
            if (!pkt_type)
                pkt_type = otp_str_data.pre_detect_pkt_num;
            slot_pair_number += pkt_cnt * pkt_type;
    }

    if (slot_pair_number > 0)
    {
            //If wifi driver doesn't set retry_index, fw will use the default value
            if (!retry_index)
                retry_index = otp_str_data.control_time_parameter & 0x0F;
    
        /* this is a factor to evaluate tx time for retransmit - austin */
            slot_pair_number = slot_pair_number*retry_index;
        return SLOT_VAL_TO_TIMER_VAL(slot_pair_number << 1);
    }
    else
    {
        return 0;
    }
}

/**
* Checks for clock wrap-around or instant-passed.
*
* \param[in] cur_clock The value of current clock.
* \param[in] future_clock The value of target clock.
*
* \return BT_CLOCK_NORMAL_CASE in normal case,
*         BT_CLOCK_CLK_WRAP_AROUND_CASE in clock wrap around case and
*         BT_CLOCK_MORE_THAN_12_HRS_AWAY at all other times.
*
* \warning Pass 28-bit clock values to this function.
*/
UCHAR lc_check_for_clock_wrap_around(UINT32 cur_clock, UINT32 future_clock)
{
    if(future_clock > cur_clock)
    {
        /* Instant in future. No clk wrap around. */
        return BT_CLOCK_NORMAL_CASE;
    }

    /* Instant is in the past. */
    if( (future_clock & (BT_CLOCK_27_BITS + 1)) ==
            (cur_clock & (BT_CLOCK_27_BITS + 1)) )
    {
        /* MSB is same, instant is more than 12 hours away. */
        return BT_CLOCK_MORE_THAN_12_HRS_AWAY;
    }
    else
    {
        /* Assumed to be clock wrap around case. */
        return BT_CLOCK_CLK_WRAP_AROUND_CASE;
    }
}

/**
* Updates the sactternet state of the device on adding an ACL connection.
*
* \param new_state SCA_MASTER or SCA_SLAVE.
*
* \return None.
*/
UINT8 lc_update_scatternet_state_addition(LC_CUR_SCATTERNET_STATE new_state,
        UCHAR piconet_id)
{
    UINT8 i = 0;

    if ((lc_sca_manager.master_cnt == 0) && (lc_sca_manager.slave_cnt == 0))
    {
        /* no any active roles in all piconets */
        i = piconet_id;
    }

    if ((lc_sca_manager.master_cnt > 0) && (new_state == SCA_MASTER))
    {
        /* No change in scatternet state. Multi-slave scenario. */
        return lc_sca_manager.master_id;
    }

    if ((lc_sca_manager.slave_cnt + lc_sca_manager.master_cnt) == MAX_PICONET_CNT)
    {
        /* no more piconets */
        return MAX_PICONET_CNT;
    }
#ifndef _DAPE_CORRECT_PICONET_INFO_WHEN_ROLE_SW
    for (; i < MAX_PICONET_CNT; i++)
    {
        if (!lc_sca_manager.pnet[i].active)
        {
            if (new_state == SCA_MASTER)
            {
                lc_sca_manager.pnet[i].master = 1;
                lc_sca_manager.master_cnt = 1;
                lc_sca_manager.master_id = i;
            }
            else
            {
                lc_sca_manager.pnet[i].master = 0;
                lc_sca_manager.slave_cnt++;
                lc_sca_manager.bm_slave |= 1 << i;
            }
            lc_sca_manager.pnet[i].active = 1;
            return i;
        }
    }

    return MAX_PICONET_CNT;
#else
    if (lmp_mss_state != LMP_MSS_INIT)
    {
        i = lmp_role_switch_data.new_piconet_id;
    }
    else
    {
        for (; i < MAX_PICONET_CNT; i++)
        {
            if (!lc_sca_manager.pnet[i].active)
            {
                break;
            }
        }
    }

    if (new_state == SCA_MASTER)
    {
        lc_sca_manager.pnet[i].master = 1;
        lc_sca_manager.master_cnt = 1;
        lc_sca_manager.master_id = i;
    }
    else
    {
        lc_sca_manager.pnet[i].master = 0;
        lc_sca_manager.slave_cnt++;
        lc_sca_manager.bm_slave |= 1 << i;
    }
    lc_sca_manager.pnet[i].active = 1;
    
    return i;
#endif
}


/**
* Updates the scatternet state of the device on removing an ACL connection.
* Make sure that lc_no_of_connections array is updated before calling this
* function.
*
* \param phy_piconet_id Physical piconet ID of the connection.
*
* \return None.
*/
void lc_update_scatternet_state_deletion(UCHAR phy_piconet_id)
{
    LC_CUR_SCATTERNET_STATE sca_state;

    sca_state = lc_get_scatternet_state_from_phy_piconet_id(phy_piconet_id);

    if (sca_state == SCA_SLAVE)
    {
        UINT8 i;
        UINT8 spid = 0;

        if (lc_sca_manager.bm_slave == 0)
        {
            return;
        }

        /* delete a slave role */
        if (lc_sca_manager.bm_slave & (1 << phy_piconet_id))
        {
            spid = phy_piconet_id;
        }
        else
        {
            for (i = 0; i < MAX_PICONET_CNT; i++)
            {
                if (lc_sca_manager.bm_slave & (1 << ((MAX_PICONET_CNT-1) - i)))
                {
                    spid = MAX_PICONET_CNT - 1 - i;
                    break;
                }
            }
        }
        lc_sca_manager.pnet[spid].active = 0;
        lc_sca_manager.bm_slave &= ~(1 << spid);
        lc_sca_manager.slave_cnt--;
    }
    else if (sca_state == SCA_MASTER)
    {
        /* delete a master role */
        if (lc_sca_manager.master_cnt)
        {
            UINT8 mpid = lc_sca_manager.master_id;
            if (lmp_self_device_data.lc_no_of_connections[mpid] == 0)
            {
                /* no connections, so we can delete it now */
                lc_sca_manager.master_cnt = 0;
                lc_sca_manager.pnet[mpid].active = 0;
                lc_sca_manager.pnet[mpid].master = 0;
            }
        }
    }

    return;
}


/**
* Resets the variables of scatternet.
*
* \param None.
*
* \return None.
*/
void lc_reset_scatternet_state(void)
{
    lc_current_scatternet_state = SCA_IDLE;
    lc_pagescan_piconet_id = SCA_PICONET_INVALID;

    UINT8 i;
    for (i = 0; i < MAX_PICONET_CNT; i++)
    {
        lmp_self_device_data.lc_no_of_connections[i] = 0;        
        lc_reset_lc_piconet_scheduler(i);
        lc_waiting_for_crc_pkt_ack[i] = FALSE;
    }
    memset(&lc_sca_manager, 0, sizeof(lc_sca_manager));

    return;
}

/**
*  Programs or kills scans based on the current scatternet state.
* Make sure that lc_current_scatternet_state and other device
* states are updated before calling this function.
*
* \param None.
*
* \return None.
*/
void lc_check_and_enable_scans_in_scatternet(void)
{
    UCHAR page_scan_status = FALSE;
    UCHAR inq_scan_status = FALSE;


#ifdef _ROM_CODE_PATCHED_
    if (rcp_lc_check_and_enable_scans_in_scatternet_func != NULL)
    {
        if (rcp_lc_check_and_enable_scans_in_scatternet_func(NULL))
        {
            return;
        }
    }
#endif

    do
    {
        if (g_lc_scan_slot_timer_in_use  == 0x1)
        {
            /* Scan timer is in use, will enable scan after the use */
            break;
        }

        if (lmp_self_device_data.scan_enable == 0)
        {
            break;
        }

        if (lmp_self_device_data.lc_cur_dev_state != LC_CUR_STATE_IDLE)
        {
            break;
        }

        /* have free piconet for use */
        if ((lc_sca_manager.master_cnt + lc_sca_manager.slave_cnt) <
                MAX_PICONET_CNT)
        {
            page_scan_status = TRUE;
            inq_scan_status = TRUE;
        }

        lc_kill_scan_mode();

        if (!IS_SUPPORT_SCATTERNET)
        {
            if (lmp_self_device_data.number_of_hlc != 0)
            {
                page_scan_status = FALSE;
            }
        }

        if (!IS_INQ_SCAN_AFTER_CONN)
        {
            if (lmp_self_device_data.number_of_hlc != 0)
            {
                inq_scan_status = FALSE;
            }
        }

#ifdef COMPILE_PARK_MODE
        if (lmp_self_device_data.number_of_parked_dev != 0)
        {
            page_scan_status = FALSE;
            inq_scan_status = FALSE;
        }
#endif

#ifdef ENABLE_SCO
        if (lmp_self_device_data.total_no_of_sco_conn != 0)
        {
            if (lmp_self_device_data.sco_pkt_type == HV3)
            {
                /* If there are two or more HV3 conn, disallow both scans. */
                if (lmp_self_device_data.total_no_of_sco_conn > 1)
                {
                    page_scan_status = FALSE;
                    inq_scan_status = FALSE;
                }
            }
            else
            {
                /* If there are one or two HV2 connection, disallow both scans. */
                /* If there is HV1 connection, disallow both scans. */
                {
                    page_scan_status = FALSE;
                    inq_scan_status = FALSE;
                }
            }
        }
#endif

#if defined(COMPILE_ESCO) || defined(ENABLE_SCO)
        if (lc_full_bandwidth_flag == TRUE)
        {
            page_scan_status = FALSE;
            inq_scan_status = FALSE;
        }
#endif

#ifdef COMPILE_ESCO
        /* Scans not allowed when there is an ESCO connection.
        Even inq-scan is not allowed. */
        //if (lmp_self_device_data.number_of_esco_connections != 0)
        if (lmp_self_device_data.number_of_esco_connections > 2)
        {
            inq_scan_status = FALSE;
            page_scan_status = FALSE;
        }
#endif

#if 0
        /* Duty cycle has to be less than 50%. */
        if(lmp_self_device_data.number_of_hlc != 0)
        {
            if( (lmp_self_device_data.page_scan_window << 1) >
                    (lmp_self_device_data.page_scan_interval) )
            {
                page_scan_status = FALSE;
                inq_scan_status = FALSE;
            }
        }
#endif

#ifdef COMPILE_HOLD_MODE
        if ((lmp_self_device_data.number_of_connections_in_hold_mode != 0) &&
                (lmp_self_device_data.hold_mode_activity !=
                 HCI_HOLD_MODE_ACTIVITY_DEFAULT) )
        {
            if (lmp_self_device_data.hold_mode_activity &
                    HCI_HOLD_MODE_ACTIVITY_SUSPEND_PAGE_SCAN)
            {
                /* Suspend page scan */
                page_scan_status = FALSE;

                //RT_BT_LOG(GRAY, LC_UTILS_3003, 0, 0);
            }

            if (lmp_self_device_data.hold_mode_activity &
                    HCI_HOLD_MODE_ACTIVITY_SUSPEND_INQ_SCAN)
            {
                /* Suspend Inq scan */
                inq_scan_status = FALSE;
                //RT_BT_LOG(GRAY, LC_UTILS_3011, 0, 0);
            }
        }
#endif

        if (lmp_mss_state != LMP_MSS_INIT)
        {
            page_scan_status = FALSE;
            inq_scan_status = FALSE;
        }

        /* Remove debug-printfs, MSS_NCTO calls this function. */
        if (page_scan_status == TRUE)
        {
            lc_retrieve_page_scan();
        }

        if (inq_scan_status == TRUE)
        {
#ifdef _3DD_FUNCTION_SUPPORT_
#ifdef _SUPPORT_CSB_TRANSMITTER_
            if (!bt_3dd_var.is_csb_mode)
#endif
            {
                if (lmp_self_device_data.inquiry_scan_type == HCI_3DD_INQ_SCAN)
                {
                    /* enable 3dd extension inquiry scan */
                    bt_3dd_driver_retrieve_inq_scan();
                    break;
                }
            }
#endif

            lc_retrieve_inq_scan();
        }
    }
    while (0);
}

/**
*  Allocates piconet_id for page-scan. Phy-piconet-ID is allocated.
*
* \param None.
*
* \return SCA_PICONET_FIRST or SCA_PICONET_SECOND.
*/
UCHAR lc_get_piconet_id_for_pagescan(void)
{
    UINT8 piconet_id;
    for (piconet_id = 0; piconet_id < MAX_PICONET_CNT; piconet_id++)
    {
        if (lc_sca_manager.pnet[piconet_id].active == 0)
        {
            break;
        }
    }
    return piconet_id;
}


/**
*  Allocates piconet_id for paging. Phy-piconet-ID is allocated.
*
* \param None.
*
* \return SCA_PICONET_FIRST or SCA_PICONET_SECOND.
*/
UCHAR lc_allocate_piconet_id_for_paging(void)
{
    UCHAR piconet_id = SCA_PICONET_INVALID;
    UCHAR allow_piconet_cnt;

    /* check we support scatternet or not */
    if (IS_SUPPORT_SCATTERNET)
    {
        allow_piconet_cnt = MAX_PICONET_CNT;
    }
    else
    {
        allow_piconet_cnt = 1;
    }

    if (lc_sca_manager.master_cnt > 0)
    {
        /* has master role in certain piconet */
        piconet_id = lc_sca_manager.master_id;
    }
    else
    {
        /* no master role, so we found free piconet for paging */
        UINT8 i;

        for (i = 0; i < allow_piconet_cnt; i++)
        {
            if (!lc_sca_manager.pnet[i].active)
            {
                piconet_id = i;
                break;
            }
        }
    }

#ifdef ENABLE_SCO
    if (lmp_self_device_data.total_no_of_sco_conn != 0)
    {
        if (lmp_self_device_data.sco_pkt_type == HV3)
        {           
            if (lmp_self_device_data.total_no_of_sco_conn == 2)
            {
                /* If there are two HV3 connection, and we are master, 
                   only then paging is allowed. If we are slave and HV3 
                   connected, then paging is not allowed.  */
                piconet_id = lc_get_master_piconet_id();
            }
            else if (lmp_self_device_data.total_no_of_sco_conn == 3)
            {
                /* If there are three HV3 connections, 
                   then paging is not allowed. */
                piconet_id = SCA_PICONET_INVALID; 
            }
        }
        else if (lmp_self_device_data.sco_pkt_type == HV2)
        {
            /* If there is a HV2 connection, and we are master, only then
            paging is allowed. If we are slave and HV2 connected, then paging
            is not allowed.  */            
            piconet_id = lc_get_master_piconet_id();            
        }
        else if (lmp_self_device_data.sco_pkt_type == HV1)
        {
            /* If there is a HV1 connection, then paging is not allowed. */
            piconet_id = SCA_PICONET_INVALID;            
        }
    }
#endif

#ifdef COMPILE_ESCO
#ifndef ALLOW_PAGING_WHEN_ESCO_AND_ALL_SLAVE_ROLE_IN_SCATTERNET
    /* If there is a ESCO connection, scatternet is not allowed.
        However, multi-slave is allowed. */
    if (lmp_self_device_data.number_of_esco_connections != 0)
    {                    
        piconet_id = lc_get_master_piconet_id();
    }
#endif    
#endif

#ifdef SCNET_DEBUG
    RT_BT_LOG(GRAY, LC_UTILS_0380_1, 5, lc_sca_manager.master_cnt,
              lc_sca_manager.master_id, lc_sca_manager.slave_cnt,
              lc_sca_manager.bm_slave, piconet_id);
#endif

    return piconet_id;
}


void lc_flush_bluewiz_fifo(UINT8 fifo_type)
{
    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();
    BB_write_baseband_register(TRANSMIT_FIFO_STATUS_REGISTER, (fifo_type << 4));
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_FLUSH);
    MINT_OS_EXIT_CRITICAL();    
}

/**
* Flushes the Tx FIFO for first piconet.
*
* \param None.
*
* \return None.
*/
INLINE void lc_flush_piconet1_tx_fifo(void)
{
    lc_flush_bluewiz_fifo(0x01);
}


/**
* Flushes the Broadcast Tx FIFO, for ACL.
*
* \param None.
*
* \return None.
*/
INLINE void lc_flush_broadcast_tx_fifo(void)
{
    lc_flush_bluewiz_fifo(0x09);
}

/**
* Flushes the ESCO rx FIFO.
*
* \param None.
*
* \return None.
*/
INLINE void lc_flush_esco_rx_fifo(void)
{
    lc_flush_bluewiz_fifo(0x0B);
}

/**
* Flushes the ESCO tx FIFO.
*
* \param None.
*
* \return None.
*/
INLINE void lc_flush_esco_tx_fifo(void)
{
    lc_flush_bluewiz_fifo(0x0A);
}


/**
* Calculates the role from physical piconet ID.
*
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return SCA_MASTER or SCA_SLAVE, for a valid piconet_id. SCA_IDLE otherwise.
*/
LC_CUR_SCATTERNET_STATE lc_get_scatternet_state_from_phy_piconet_id(
    UCHAR phy_piconet_id)
{
    LC_CUR_SCATTERNET_STATE return_state;

    if (lc_sca_manager.pnet[phy_piconet_id].active)
    {
        if (lc_sca_manager.pnet[phy_piconet_id].master)
        {
            return_state = SCA_MASTER;
        }
        else
        {
            return_state = SCA_SLAVE;
        }
    }
    else
    {
        return_state = SCA_IDLE;
    }

    return return_state;

}

/**
* Allocates new piconet_id for Role Switch.
*
* \param None.
*
* \return TRUE if the device is in more than one piconet, FALSE otherwise.
*/
UCHAR lc_check_if_device_is_in_scatternet(void)
{
    UCHAR sca_status = FALSE;

    if ((lc_sca_manager.master_cnt + lc_sca_manager.slave_cnt) > 1)
    {
        sca_status = TRUE;
    }

    return sca_status;

}

/**
* Calculates the lut_index from the physical piconet ID.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return lut_index of the connection.
*/
UCHAR lc_get_lut_index_from_phy_piconet_id(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index = 0;

    lut_index = am_addr;

    if (phy_piconet_id <= SCA_PICONET_MAX)
    {
        if ((lc_sca_manager.pnet[phy_piconet_id].active == 1) &&
                (lc_sca_manager.pnet[phy_piconet_id].master == 0))
        {
            lut_index = LC_SCA_SLAVE_1_LUT + phy_piconet_id;
        }
    }

    return lut_index;
}

/**
* Calculates the lut_index from the physical piconet ID.
*
* \param am_addr The am_addr of the connection.
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return lut_index of the connection.
*/
UCHAR lc_get_lut_index_from_phy_piconet_id_for_disable_afh(UCHAR am_addr, UCHAR phy_piconet_id)
{
    UCHAR lut_index = 0;

    lut_index = am_addr;

    if (phy_piconet_id <= SCA_PICONET_MAX)
    {
        if (lc_sca_manager.pnet[phy_piconet_id].master == 0)
        {
            lut_index = LC_SCA_SLAVE_1_LUT + phy_piconet_id;
        }
    }

    return lut_index;
}

/**
* Returns the role of the device for a particular piconet-ID.
*
* \param phy_piconet_id Phy piconet ID of the connection.
*
* \return MASTER or SLAVE.
*/
UCHAR lc_sca_get_piconet_role(UCHAR phy_piconet_id)
{
    UCHAR sel_role = MASTER;

    if (phy_piconet_id <= SCA_PICONET_FOURTH)
    {
        if (lc_sca_manager.pnet[phy_piconet_id].active)
        {
            if (lc_sca_manager.pnet[phy_piconet_id].master)
            {
                sel_role = MASTER;
            }
            else
            {
                sel_role = SLAVE;
            }
        }
    }

    return sel_role;
}

#ifdef COMPILE_ROLE_SWITCH
/**
* Allocates lut_index for Role Switch.
*
* \param ce_index Index to the lmp-connection-entity.
*
* \return SCA_PICONET_FIRST or SCA_PICONET_SECOND.
*
*/
UCHAR lc_allocate_piconet_id_for_mss(UINT16 ce_index)
{
    UCHAR new_piconet_id = 0;
    UINT8 pnet_cnt = lc_sca_manager.master_cnt + lc_sca_manager.slave_cnt;

    if (pnet_cnt == 0)
    {
        /* SCA_IDLE */
        /* (this maybe is an error case !!) */
        new_piconet_id = SCA_PICONET_FIRST;
    }
    else if (pnet_cnt == 1)
    {
        /* For single piconet */
        if (lc_sca_manager.master_cnt > 0)
        {
            /* we only have one master role in a piconet */
            if(lmp_self_device_data.lc_no_of_connections[lc_sca_manager.master_id] > 1)
            {
                /* search new piconet for role switch */
                new_piconet_id = lc_sca_manager.master_id;
                
                if (new_piconet_id == SCA_PICONET_FIRST)
                {
                    new_piconet_id = SCA_PICONET_SECOND;
    			    AND_val_with_bb_reg(PICONET2_INFO_REGISTER, ~(BIT0));                        
                }
                else
                {
                    new_piconet_id = SCA_PICONET_FIRST;
                    AND_val_with_bb_reg(PICONET1_INFO_REGISTER, ~(BIT0));  
                }
            }
            else
            {
                new_piconet_id = SCA_PICONET_FIRST;
                if (lc_sca_manager.master_id != SCA_PICONET_FIRST)
                {
            		AND_val_with_bb_reg(PICONET1_INFO_REGISTER, ~(BIT0));                        
                }
            }
        }
        else
        {
            /* we only have one slave role in a piconet */

            new_piconet_id = SCA_PICONET_FIRST;

            if (!(lc_sca_manager.bm_slave & BIT0))
            {
	            OR_val_with_bb_reg(PICONET1_INFO_REGISTER,BIT0);                    
            }               
        }            
    }
    else
    {
        /* for multiple piconets */
        if (lc_sca_manager.master_cnt)
        {
            new_piconet_id = lc_sca_manager.master_id;
        }
        else
        {
            new_piconet_id = lmp_connection_entity[ce_index].phy_piconet_id;
        }            
    }

    return new_piconet_id;
}


/**
* Allocates lut_index for Role Switch.
*
* \param ce_index Index to the lmp-connection-entity.
* \param new_piconet_id The new piconet ID of the connection after MSS.
* \param new_am_addr The new_am_addr of the connection.
*
* \return am_addr The allocated lut_index.
*
*/
UCHAR lc_allocate_lut_index_for_mss(UINT16 ce_index,
                                    UCHAR new_piconet_id, UCHAR new_am_addr)
{
    UCHAR new_lut_index = 0;
    UCHAR lcl_dev_role;
    LMP_CONNECTION_ENTITY *ce_ptr;

    ce_ptr = &lmp_connection_entity[ce_index];

    lcl_dev_role = (ce_ptr->remote_dev_role) ^ 0x01;

    if (lcl_dev_role == SLAVE)
    {
        new_lut_index = new_am_addr;
    }
    else
    {
        if(new_piconet_id <= SCA_PICONET_MAX)
        {
            new_lut_index = LC_SCA_SLAVE_1_LUT + new_piconet_id;
        }
        else
        {
            RT_BT_LOG(RED, PICONET_ID_CHECK_MSG, 1, new_piconet_id);               
        }
    }

    return new_lut_index;
}


/**
* Allocates am_addr for MSS; this is called only when switching
* from slave to master.
*
* \param ce_index Index to the lmp-connection-entity.
*
* \return am_addr The allocated LT-addr.
*
*/
UCHAR lc_allocate_am_addr_for_mss(UINT16 ce_index, UCHAR new_piconet_id)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UCHAR am_addr;
    UCHAR old_piconet_id;

    am_addr = INVALID_AM_ADDR;

    ce_ptr = &lmp_connection_entity[ce_index];

    old_piconet_id = ce_ptr->phy_piconet_id;

    if (ce_ptr->remote_dev_role == MASTER)
    {
        /* If the same am_addr is available in the  */
        if (old_piconet_id == new_piconet_id)
        {
            RT_BT_LOG(GRAY, LC_UTILS_3673, 2, ce_ptr->am_addr, new_piconet_id);
            return ce_ptr->am_addr;
        }

        lmp_get_am_addr_ppi(&am_addr, new_piconet_id);

        /* added by cch */
        lmp_am_addr_to_ce_index_table_ppi[am_addr-1][new_piconet_id].ce_index = ce_index;
    }

    return am_addr;
}

/**
* Updates upper and lower LUT addresses of the new-lut in lut-ex-table
*   for MSS.
*
* \param None.
*
* \return None.
*
* \note: Call this function after updating
*  lmp_role_switch_data.new_lut_index variable.
*
*/
void lc_update_addresses_in_lut_ex_table_for_mss()
{
    UCHAR new_lut_index;
    UINT16 lcl_new_lower_lut_address;
    UINT16 lcl_new_upper_lut_address;

    new_lut_index = lmp_role_switch_data.new_lut_index;

    if ((new_lut_index >= LC_SCA_SLAVE_1_LUT) &&
        (new_lut_index <= LC_SCA_SLAVE_4_LUT))
    {
        UINT8 idx = new_lut_index - LC_SCA_SLAVE_1_LUT;
        lcl_new_lower_lut_address = reg_SCA_SLAVE_LOWER_LUT[idx];
        lcl_new_upper_lut_address = reg_SCA_SLAVE_UPPER_LUT[idx];
    }
    else
    {
        lcl_new_lower_lut_address = reg_MASTER_LOWER_LUT[new_lut_index];
        lcl_new_upper_lut_address = reg_MASTER_UPPER_LUT[new_lut_index];
    }

    lmp_role_switch_data.new_lower_lut_address = lcl_new_lower_lut_address;
    lmp_role_switch_data.new_upper_lut_address = lcl_new_upper_lut_address;

    return;
}

/**
* Updates the scatternet status after successful MSS.
*
* \param ce_index Index to the lmp-connection-entity.
*
* \return None.
*
*/
void lc_update_scatternet_status_after_mss(UINT16 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;    
    UCHAR old_piconet_id;
    UCHAR new_piconet_id;
    UCHAR new_am_addr;

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    old_piconet_id = lmp_role_switch_data.old_piconet_id;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;
    new_am_addr = lmp_role_switch_data.new_am_addr;

    ce_ptr = &lmp_connection_entity[ce_index];

    if ((old_piconet_id > SCA_PICONET_MAX) || 
            (new_piconet_id > SCA_PICONET_MAX)) 
    {
        MINT_OS_EXIT_CRITICAL();
        RT_BT_LOG(RED, PICONET_ID_CHECK_MSG_NEW, 2, 
                                old_piconet_id, new_piconet_id);          
        return;
    }

    /* Update the no of connections in old piconet. */

    if (lmp_self_device_data.lc_no_of_connections[old_piconet_id] != 0)
    {
        lmp_self_device_data.lc_no_of_connections[old_piconet_id]--;
    }

    if (old_piconet_id != new_piconet_id)
    {
        lc_check_and_cleanup_bd_addr_regs_on_disc(old_piconet_id);
    }

    /* Update the scatternet state of the device. */
    lc_update_scatternet_state_deletion(old_piconet_id);

    if (lmp_self_device_data.lc_no_of_connections[old_piconet_id] == 0)
    {
        AND_val_with_bb_reg_isr(reg_PICONET_INFO[old_piconet_id],
                                (UINT16)(~0x03));
        BB_write_baseband_register(INSTRUCTION_REGISTER, BB_EXECUTE);        
    }

    /* Update the no of connections in new piconet. */
    lmp_self_device_data.lc_no_of_connections[new_piconet_id]++;

    /* Note: ce_ptr->remote_dev_role is still the old role. */
    /* Update Piconet Info register, and clean up old register. */
    if (ce_ptr->remote_dev_role == MASTER)
    {
        BZ_REG_S_PICONET_INFO temp;
        UINT16 addr;
        lc_update_scatternet_state_addition(SCA_MASTER, new_piconet_id);
        addr = reg_PICONET_INFO[new_piconet_id];        
        *(UINT16*)&temp = BB_read_baseband_register(addr);
        temp.lt_addr = BC_AM_ADDR;
        temp.master = TRUE;
        BB_write_baseband_register(addr, *(UINT16*)&temp);
    }
    else if (ce_ptr->remote_dev_role == SLAVE)
    {
        BZ_REG_S_PICONET_INFO temp;
        UINT16 addr;
        lc_update_scatternet_state_addition(SCA_SLAVE, new_piconet_id);
        addr = reg_PICONET_INFO[new_piconet_id];     
        *(UINT16*)&temp = BB_read_baseband_register(addr);
        temp.lt_addr = new_am_addr;
        temp.master = FALSE;
        BB_write_baseband_register(addr, *(UINT16*)&temp);
    }

    MINT_OS_EXIT_CRITICAL();

    return;
}
#endif /* COMPILE_ROLE_SWITCH */

/**
* Flushes the Rx FIFO, of ACL.
*
* \param None.
*
* \return None.
*/
void lc_flush_rx_fifo(void)
{
    DEF_CRITICAL_SECTION_STORAGE;
    MINT_OS_ENTER_CRITICAL();

    /* To be implemented. */
    BB_write_baseband_register(TRANSMIT_FIFO_STATUS_REGISTER, (0x0 << 4));
    BB_write_baseband_register(INSTRUCTION_REGISTER, BB_FLUSH);

    MINT_OS_EXIT_CRITICAL();

#ifdef SCNET_DEBUG
    LMP_LOG_INFO(LOG_LEVEL_HIGH, "Flushed Rx-fifo.");
#endif

    return;
}

/**
* Sets the lc_cur_connecting_am_addr variable.
*
* \param val The value to be set.
*
* \return None.
*/
INLINE void lc_set_lc_cur_connecting_am_addr(UCHAR val)
{
    lc_cur_connecting_am_addr = val;
}


/**
* Computes the number of slots (clk1 ticks) till the first instant. The
* calculation is based on the current clock. The returned value can be
* directly programmed into the corresponding SLOT_OFFSET registers of the
* baseband.
*
* \param[in] Tinterval The interval at which the event recurs (eg. Tsco, Tesco,
*                      Tsniff, etc.)
* \param[in] Dphase The phase in the interval at which the event occurs (eg.
*                   Dsco, Desco, Dsniff, etc.)
* \param[in] tcf Timing control flag (except BIT1, the rest are don't care. If
*                the BIT1 is set then the initialization procedure 2 is used,
*                otherwise initialization procedure 1 is used).
* \param[in] piconet_id Piconet ID of the connection for which slots are
*                       computed.
* \param[out] nslots_to_instant The computed number of slots (based on the
*                               above parameters) to the first instant. This
*                               will never be zero or greater than Tinterval.
*
* \return None.
*
* \note This procedure takes care of the clock wraparound issue.
*/
UINT32 lc_get_nslots_to_first_instant(UINT16 Tinterval, UINT16 Dphase, UCHAR tcf,
                                    UCHAR piconet_id, UINT16* nslots_to_instant)
{
    UINT32 working_clock;
    UINT16 offset;

    lc_get_clock_in_scatternet(&working_clock, piconet_id);

    if (tcf & BIT1)    /* Use initialization procedure 2 */
    {
        /* clk(!(27), 26:1) */
        working_clock = ((working_clock ^ BIT27) >> 1) & BITMASK_27;
    }
    else    /* Use initialization procedure 1 */
    {
        working_clock = (working_clock >> 1) & BITMASK_27; /* clk27:1 */
    }

    offset = (UINT16)(working_clock % Tinterval);

    if (Dphase == offset)
    {
        /* Buffer period for the BB instead of performing the action
        * immediately. Don't want the nslots_to_instant to be 0.
        */
        *nslots_to_instant = Tinterval;
    }
    else if (Dphase > offset)
    {
        *nslots_to_instant = (UINT16)(Dphase - offset);
    }
    else
    {
        *nslots_to_instant = (UINT16)((Tinterval - offset) + Dphase);
    }

    return working_clock;
}

/**
* Spins until the clk1 transition happens.
*
* \param[in] piconet_id The piconet ID of the piconet clock under
*                       consideration.
*
* \return The clock value after transition.
*
* \note This procedure takes care of the clock wraparound issue.
*/
UINT32 lc_spin_until_clk1_transition(UCHAR piconet_id)
{
    UINT32 clk;
    UCHAR bit1;

    lc_get_clock_in_scatternet(&clk, piconet_id);

    bit1 = (UCHAR)(clk & BIT1);

    do
    {
        lc_get_clock_in_scatternet(&clk, piconet_id);
    }
    while ((clk & BIT1) == bit1);

    return clk;
}

/**
* Calculates the piconet_id of the established connection.
*
* \param None.
*
* \return SCA_PICONET_FIRST for the first piconet, SCA_PICONET_SECOND
* for the second piconet, and SCA_PICONET_INVALID if there is no connection.
*/
UCHAR lc_get_connected_piconet_id(void)
{
    UCHAR ret_val = SCA_PICONET_INVALID;

    if ((lc_sca_manager.master_cnt + lc_sca_manager.slave_cnt) == 1)
    {
        UINT8 i;
        for (i = 0; i < MAX_PICONET_CNT; i++)
        {
            if (lc_sca_manager.pnet[i].active)
            {
                ret_val = i;
                break;
            }
        }
    }

    return ret_val;
}

/**
* Calculates the number of piconets the device is currently connected.
*
* \param None.
*
* \return 0, 1 or 2 accordingly.
*/
INLINE UCHAR lc_get_no_of_piconets_connected(void)
{
    return (lc_sca_manager.master_cnt + lc_sca_manager.slave_cnt);
}

#ifdef COMPILE_ROLE_SWITCH
/**
* Checks if packets are scheduled for the MSS connection at the
* switch instant. If present, aborts the MSS procedure. This function
* is called just before the instant.
*
* \param phy_piconet_id The piconet_id of the connection.
*
* \return TRUE if MSS needs to be aborted, FALSE otherwise.
*/
UCHAR lc_check_scheduler_for_aborting_mss(UCHAR phy_piconet_id)
{
    LC_SCHEDULED_PKT *schd;
    LC_PICONET_SCHEDULER *piconet_schd;
    UINT16 lcl_ce_index;
    UCHAR mss_abort = FALSE;

    lcl_ce_index = lmp_role_switch_data.ce_index;

    piconet_schd = &lc_piconet_scheduler[phy_piconet_id];

    LC_SCHEDULED_PKT *schd1;
    schd = &piconet_schd->lc_scheduled_pkt_info[piconet_schd->rptr];
    schd1 = &piconet_schd->lc_scheduled_pkt_info[!piconet_schd->rptr];

    switch(piconet_schd->lc_allowed_pkt_cnt)
    {
        case 2: /* No entries in scheduler. */
            break;

        case 1: /* One packet loaded in scheduler. */
            /* Check if the loaded packet is for the MSS connection. */
            if(schd->ce_index == lcl_ce_index)
            {
                mss_abort = TRUE;
            }
            break;

        case 0: /* Two packets are loaded into scheduler. */
            /* Check if the loaded packet is for the MSS connection. */
            if( (schd->ce_index == lcl_ce_index) || 
                (schd1->ce_index == lcl_ce_index) )
            {
                mss_abort = TRUE;
            }
            break;

        default: /* Invalid case. */            
            break;
    }

    if(mss_abort == TRUE)
    {
        RT_BT_LOG(GRAY, LC_UTILS_4121, 0, 0);
    }

    return mss_abort;
}


/**
* Cleans up the data strcutures after aborting MSS. This cleanup is required
* when LM negotiation is successful and MSS procedure has started, and LC is
* spinning for the instant. Just before the instant, if the scheduler has
* packets queued for the MSS connection, LC will decice to abort the MSS
* procedure. Then this function will cleanup the MSS data structures.
*
* \param None.
*
* \return None.
*/
void lc_cleanup_mss_abort_at_instant()
{
    UINT16 ce_index;
    UCHAR new_am_addr;
    UCHAR new_piconet_id;
    UCHAR old_am_addr;
    UCHAR old_piconet_id;

    ce_index = lmp_role_switch_data.ce_index;
    new_am_addr = lmp_role_switch_data.new_am_addr;
    new_piconet_id = lmp_role_switch_data.new_piconet_id;
    old_am_addr = lmp_role_switch_data.old_am_addr;
    old_piconet_id = lmp_role_switch_data.old_piconet_id;

    if ( (lmp_connection_entity[ce_index].remote_dev_role == MASTER) &&
            (old_am_addr != new_am_addr) &&
            (old_piconet_id != new_piconet_id) )
    {
        lmp_slave_unuse_am_addr_ppi(new_am_addr, new_piconet_id);
    }

    lmp_set_mss_state(LMP_MSS_INIT);

    lmp_handle_role_switch_failure(ce_index, ROLE_SWITCH_FAILED);

    if (g_lc_scan_slot_timer_in_use == 0x1)
    {
        g_lc_scan_slot_timer_in_use = 0x0;
        /* Configure & Retrieve the scan */
        lc_handle_scan_mode_command();
    }
    else
    {
        lc_check_and_enable_scans_in_scatternet();
    }

    return;
}
#endif /* COMPILE_ROLE_SWITCH */

/**
* Cleans up the remote device BD-addr registers on disconnection. This is
* also done on MSS success. Make sure that this function is called after
* updating lc_no_of_connections.
*
* \param phy_piconet_id The piconet_id of the connection.
*
* \return None.
*/
void lc_check_and_cleanup_bd_addr_regs_on_disc(UCHAR phy_piconet_id)
{
#ifdef COMPILE_PARK_MODE
    if(lmp_self_device_data.number_of_parked_dev != 0)
    {
        return;
    }
#endif

    if (phy_piconet_id > SCA_PICONET_MAX)
    {
        return;
    }

    if (lmp_self_device_data.lc_no_of_connections[phy_piconet_id] == 0)
    {
        UINT16 lcl_remote_dev_address1;
        UINT16 lcl_remote_dev_address2;
        UINT16 lcl_remote_dev_address3;
        UINT16 temp_val;

        lcl_remote_dev_address1 = reg_PICONET_BD_ADDR1[phy_piconet_id];
        lcl_remote_dev_address2 = reg_PICONET_BD_ADDR2[phy_piconet_id];
        lcl_remote_dev_address3 = reg_PICONET_BD_ADDR3[phy_piconet_id];

        temp_val = BB_read_baseband_register(lcl_remote_dev_address1);
        temp_val = temp_val ^ 0xffff;
        BB_write_baseband_register(lcl_remote_dev_address1, temp_val);

        temp_val = BB_read_baseband_register(lcl_remote_dev_address2);
        temp_val = temp_val ^ 0xffff;
        BB_write_baseband_register(lcl_remote_dev_address2, temp_val);

        temp_val = BB_read_baseband_register(lcl_remote_dev_address3);
        temp_val = temp_val ^ 0xffff;
        BB_write_baseband_register(lcl_remote_dev_address3, temp_val);
    }

    return;
}

#ifdef  COMPILE_ROLE_SWITCH
/**
* Starts the slot timer for role switch. This function checks the
* distance to the switch instant, and decides whether to start the
* timer or not.
*
* \param ce_index Index to the lmp_connection_entity, of the connection.
*
* \return 0x0 Proceed with MSS now, without starting timer.
*         0x1 Star the timer, but pause ACL data.
*         0x2 Star the timer, dont pause ACL data.
*/
UCHAR lmp_start_switch_instant_timer(UINT16 ce_index)
{
#ifdef USE_SLOT_TIMER_FOR_ROLE_SWITCH

    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT32 lcl_switch_instant;
    UINT32 cur_clk;
    UCHAR lcl_pic_id;
    UINT32 lcl_diff = 0x0;

    ce_ptr = &lmp_connection_entity[ce_index];

    lcl_switch_instant = ce_ptr->switch_instant;
    lcl_pic_id = ce_ptr->phy_piconet_id;

    /* Calculate the difference. */
    lc_get_clock_in_scatternet(&cur_clk, lcl_pic_id);

    if (lc_check_for_clock_wrap_around( cur_clk,
                                        (lcl_switch_instant << 1) ) == BT_CLOCK_CLK_WRAP_AROUND_CASE)
    {
        cur_clk = (cur_clk | (BT_CLOCK_27_BITS + 1));
    }

    cur_clk = cur_clk >> 1;

    if (cur_clk < lcl_switch_instant)
    {
        lcl_diff = lcl_switch_instant - cur_clk;
    }

    /* Guard time of LC_MSS_MIN_SLOT_DIFF => 4 slots. */
    if (lcl_diff <= (LC_MSS_MIN_SLOT_DIFF + 2))
    {
        /* proceed with MSS immediately */
        return 0x0;
    }

    if (lcl_diff > (LC_MSS_CAN_WAIT_MIN_SLOT_DIFF + LC_MSS_MIN_SLOT_DIFF))
    {
        /* Wait for MSS, Do not pause data transfer now */
        lc_program_and_start_slot_counter(lcl_diff -
                                          LC_MSS_CAN_WAIT_MIN_SLOT_DIFF);
        return 0x2;
    }
    else
    {
        /* Wait for MSS, but pause data transfer now */
        lc_program_and_start_slot_counter(lcl_diff - LC_MSS_MIN_SLOT_DIFF);
        return 0x1;
    }
#else
    return 0x0;
#endif
}

void lc_program_and_start_slot_counter(UINT32 duration)
{
    lc_kill_scan_mode();

    g_lc_scan_slot_timer_in_use = 0x1;

    /* Clear all the higher bits. */
    AND_val_with_bb_reg(PAGE_SCAN_WINDOW_REGISTER, (UINT16)(~0xf000));

    /* Program duration. The duration in 1.25 ms */
    BB_write_baseband_register(PAGE_SCAN_INTERVAL_REGISTER, (duration>>1));

    /* One shot. */
    OR_val_with_bb_reg(PAGE_SCAN_WINDOW_REGISTER, BIT14);

    /* Enable the timer. */
    OR_val_with_bb_reg(PAGE_SCAN_WINDOW_REGISTER, BIT13);

    /* Start the timer. */
    OR_val_with_bb_reg(PAGE_SCAN_WINDOW_REGISTER, BIT15);

    /* Done. now wait for interrupt. */

    //LMP_LOG_INFO(LOG_LEVEL_HIGH, "Pgmed slot counter for 0x%x.", duration);

    return;
}

/**
* Checks if MSS has to be aborted. This function will check all the
* connections, whether there is a overlapping sniff instant, or ACL packets
* still pending in the scheduler to be transmitted and decides whether to
* abort or proceed with the MSS prodecure.
*
* \param None.
*
* \return TRUE if MSS has to be aborted, FALSE otherwise.
*/
UCHAR lc_check_for_aborting_mss(UCHAR piconet_id)
{
    UCHAR ret_val;

    ret_val = lc_check_scheduler_for_aborting_mss(piconet_id);

    return ret_val;
}
#endif

/**
* Reset the particular entry of the ACL pkt scheduler. To be called on
* initialization and on MSS.
*
* \param index Index to lc_piconet_scheduler to be cleaned up.
*
* \return None.
*/
void lc_reset_lc_piconet_scheduler(UINT32 index)
{
    LC_PICONET_SCHEDULER  *piconet_schd;
    UINT8 i;

    piconet_schd = &lc_piconet_scheduler[index];
    piconet_schd->rptr = 0;
    piconet_schd->wptr = 0;
    piconet_schd->lc_allowed_pkt_cnt = LC_MAX_SCH_INFO;
    piconet_schd->donot_schedule_pkt = 0x0;
    piconet_schd->wait_for_clear_pkt_count = 0x0;

    for(i = 0; i < LC_MAX_SCH_INFO; i++)
    {
        piconet_schd->lc_scheduled_pkt_info[i].tx_status = LC_TX_IDLE;
    }

    return;
}

void lc_stop_tpoll_on_all_connections_except(UINT32 ce_index)
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 temp_var;

    for (temp_var = 0; temp_var < LMP_MAX_CE_DATABASE_ENTRIES; temp_var++)
    {
        UINT32 am_addr, pid;

        if (temp_var == ce_index)
        {
            continue;
        }

        ce_ptr = &lmp_connection_entity[temp_var];

        if (ce_ptr->entity_status == ASSIGNED)
        {
            UINT32 lut_index;
            DEF_CRITICAL_SECTION_STORAGE;

            am_addr = ce_ptr->am_addr;
            pid = ce_ptr->phy_piconet_id;

#ifdef _DAPE_TEST_CHK_LUT
            if (lc_sca_manager.pnet[pid].active == 0)
            {
                continue;
            }
#endif

            lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, pid);

            /* Check if the Tpoll is running for the connection. */
            if(lc_is_tpoll_started[lut_index] == TRUE)
            {
                BB_stop_tpoll(am_addr, pid);

                /* Update the pkt type in LUT to invalid.
                 * This will be done only as master. Only
                 * master can have Poll in LUT.
                 */
                lc_check_and_update_pkt_in_lut(
                    LC_MASTER_DEFAULT_PACKET_TYPE, LC_INVALID_PACKET_TYPE,
                    lut_ex_table[lut_index].lower_lut_address);

                MINT_OS_ENTER_CRITICAL();
                lc_cont_crc_rx_cnt[lut_index] = 0xff;
                lc_cont_crc_tx_cnt[lut_index] = 0xff;
                MINT_OS_EXIT_CRITICAL();

                LMP_LOG_INFO(LOG_LEVEL_TRACE, LC_UTILS_MSS_STOP_TPOLL, 2, am_addr, pid);
            }
        }
    }

    return;
}


void lc_start_tpoll_on_all_connections()
{
    LMP_CONNECTION_ENTITY *ce_ptr;
    UINT8 temp_var;
    DEF_CRITICAL_SECTION_STORAGE;

    for (temp_var = 0; temp_var < LMP_MAX_CE_DATABASE_ENTRIES; temp_var++)
    {
        UINT8 am_addr;
        UINT8 pid;
        UINT16 tpoll;

        ce_ptr = &lmp_connection_entity[temp_var];

        if (ce_ptr->entity_status == ASSIGNED)
        {
            UINT8 lut_index;

            am_addr = ce_ptr->am_addr;
            pid = ce_ptr->phy_piconet_id;
            lut_index = lc_get_lut_index_from_phy_piconet_id(am_addr, pid);

            MINT_OS_ENTER_CRITICAL();
            lc_cont_crc_rx_cnt[lut_index] = 0x00;
            lc_cont_crc_tx_cnt[lut_index] = 0x00;
            MINT_OS_EXIT_CRITICAL();

            if ((ce_ptr->in_sniff_mode == FALSE) &&
                    (lc_is_tpoll_started[lut_index] == FALSE) 
#ifdef _DAPE_TEST_CHK_LUT
               && (lc_sca_manager.pnet[pid].active == 1)
#endif
#ifdef _DAPE_DONT_START_TPOLL_ON_CONNECTION_WHEN_NOT_CONNECTED
               &&(ce_ptr->ce_status != LMP_PAGING)
               && (ce_ptr->ce_status != LMP_DURING_CONN_REMOTE_NAME_REQ)
#endif

			)
            {
                tpoll = ce_ptr->Tpoll;
                BB_start_tpoll(am_addr, tpoll, pid);

                LMP_LOG_INFO(LOG_LEVEL_TRACE, LC_UTILS_MSS_START_TPOLL, 3, 
                                                    am_addr, tpoll, pid);
            }
        }
    }

    return;
}

