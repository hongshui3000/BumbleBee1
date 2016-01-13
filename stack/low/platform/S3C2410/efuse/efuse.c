/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 212 };

#include "efuse.h"
#include "otp.h"
#include "platform.h"
#include "DataType.h"

#if defined(_BT_ONLY_) || defined(_NEW_BTON_DESIGN_AFTER_RTL8821B_TC_)
#include "new_io.h"
#endif

#include "mem.h"

/**
 *   EFUSE Physical header format 
 *   One byte header format: 
 *       header: byte [7:4] => 0, 1, 2, 3, ... ,F (total 15)
 *                    byte [3:0] => wold bit map 1: doesn't update, 0: update
 *       
 *       total header:
 *               0x00, 0x10, ....0xF0
 *
 *       ex: 0x12
 *           header: 0x1
 *            word bit map: 4b'0010 => the second word need to be update
 *
 *   Two byte header format: byte1, byte2
 *       header: byte1 [7:4] => 0, 2, 4, 6, 8, A, C, E (total 8) (small section)
 *               byte1 [3:0] => constent 0xF
 *               byte1 => 0x0F, 0x2F, 0x4F, 0x6F, 0x8F, 0xAF, 0XCF, 0xEF
 *               byte2 [7:4] => 2, 3, ..., F (total 14) (large section)
 *               byte2 [3:0] => word bit map 1: doesn't update, 0: update
 *
 *       total header:
 *               0x0F 0x20, 0x2F 0x20, 0x4F 0x20, 0x6F 0x20,
 *               0x8F 0x20, 0xAF 0x20, 0xCF 0x20, 0xEF 0x20,
 *               -------------------------------------------
 *               0x0F 0x30, 0x2F 0x30, 0x4F 0x30, 0x6F 0x30,
 *               0x8F 0x30, 0xAF 0x30, 0xCF 0x30, 0xEF 0x30,
 *               -------------------------------------------
 *                                  .
 *                                  .
 *               -------------------------------------------
 *               0x0F 0xF0, 0x2F 0xF0, 0x4F 0xF0, 0x6F 0xF0,
 *               0x8F 0xF0, 0xAF 0xF0, 0xCF 0xF0, 0xEF 0xF0,
 *               -------------------------------------------
 */

/**
 * Function     : efuse_one_byte_read
 *
 * Description  : This function is used to read efuse content.
 *
 * Parameters : addr: reade address
 *                      data: place data point
 *
 * Returns       : None
 *
 * Side Effects : None
 *
 */
UINT8 efuse_one_byte_read(UINT16 addr, UCHAR* data)
{
    UINT8 tmpidx = 0;

    EFUSE_BYTE_WRITE((EFUSE_CTL1_REG + 1),(UINT8) (addr & 0xff));
    EFUSE_BYTE_WRITE((EFUSE_CTL1_REG + 2),((UINT8) ((addr >> 8) & 0x3)) | 
                    (EFUSE_BYTE_READ(EFUSE_CTL1_REG + 2) & 0xFC));
    EFUSE_BYTE_WRITE((EFUSE_CTL1_REG + 3), 0x0F);

    /* wait bit[7] of 0x147 to toggle */
    while (!(0x80 & EFUSE_BYTE_READ(EFUSE_CTL1_REG + 3)))
    {                
        tmpidx++;
    
        if (tmpidx > 100)
        {
            *data = 0xFF;
            return FALSE;
        }
    }

    *data = EFUSE_BYTE_READ(EFUSE_CTL1_REG);
    return TRUE;
}
 
     
 /**
  * Function     : start_update_parameter_from_efuse
  *
  * Description  : This function is used to update efuse content.
  *
  * Parameters   : None.
  *
  * Returns      : None
  *
  * Side Effects : None
  *
  */
void start_update_parameter_from_efuse(void)
{
    UINT8 check_efuse_enable;
    
    /* select efuse bank 1 */
    EFUSE_BYTE_WRITE((EFUSE_CTL2_REG + 3), EFUSE_BANK_1);

#ifdef _IS_ASIC_  
    /* read physical address of efuse map to check efuse map */
    efuse_one_byte_read(0, &check_efuse_enable); 
    
    if (check_efuse_enable != 0xFF)
#else
    /* read physical address of efuse map to check efuse map */
    efuse_one_byte_read(4, &check_efuse_enable); 

    if (check_efuse_enable == 0xA5)
#endif
    {
#ifndef  _REDUCE_BT_EFUSE_1K_
        /* read all efuse data from bank 1 */
        read_efuse_bank_data(512);

        /* select efuse bank 2 */        
        EFUSE_BYTE_WRITE((EFUSE_CTL2_REG + 3), EFUSE_BANK_2);
        /* read all efuse data from bank 2 */
        read_efuse_bank_data(512);

#ifndef _RTL8821A_
        /* select efuse bank 3 */
        EFUSE_BYTE_WRITE((EFUSE_CTL2_REG + 3), EFUSE_BANK_3);
        /* read all efuse data from bank 3 */
        read_efuse_bank_data(512);
#endif

#else
        /* read all efuse data from bank 1 */
        read_efuse_bank_data(128);

#endif /* end of #ifndef  _REDUCE_BT_EFUSE_1K_ */
    }
    
#ifndef _IS_ASIC_
#ifdef _YL_GPIO_POW_TEST_FORCE_EFUSE /* yilinli test */
    gpio_pow_test_force_efuse();
#endif
#endif
    
}


/**
    total header:
            0x00, 0x10, ....0xE0

    total header:
            byte1[7:4][3:0] byte2[7:4][3:0]
            byte1[7:4]: small section
            byte2[7:4]: large section
            -------------------------------------------
            0x0F 0x20, 0x2F 0x20, 0x4F 0x20, 0x6F 0x20,
            0x8F 0x20, 0xAF 0x20, 0xCF 0x20, 0xEF 0x20,
            -------------------------------------------
            0x0F 0x30, 0x2F 0x30, 0x4F 0x30, 0x6F 0x30,
            0x8F 0x30, 0xAF 0x30, 0xCF 0x30, 0xEF 0x30,
            -------------------------------------------
                               .
                               .
            -------------------------------------------
            0x0F 0xF0, 0x2F 0xF0, 0x4F 0xF0, 0x6F 0xF0,
            0x8F 0xF0, 0xAF 0xF0, 0xCF 0xF0, 0xEF 0xF0,
            -------------------------------------------
    sram efuse parameter[]: it doesn't include efuse header(1 byte or 2 bytes)
*/

/**
 * Function     : read_efuse_bank_data
 *
 * Description  : This function is used to check if that the efuse content is
 *                      need to be updated.
 *
 * Parameters   : size -- actual byte size of efuse bank.
 *
 * Returns      : None
 *
 * Side Effects : None
 *
 */
void read_efuse_bank_data(UINT16 size)
{    
    UINT8 efuse_data;
    UINT8 word_enable_map; 
    UINT8 word_index; 
    UINT16 *p_efuse;
    UINT32 efuse_header_index;
    UINT16 efuse_read_index = 0;
    UINT16 word_offset;
    UINT8 small_section;
    UINT8 large_section;
    UINT8 overflow = 0;
    UINT16 word_tmp;
    
    p_efuse = (UINT16 *) &otp_str_data;

    while (efuse_read_index < size)
    {
        /* parser 1st byte of header */        
        efuse_one_byte_read(efuse_read_index, &efuse_data);
        efuse_read_index++;

        if ((efuse_data == 0xFF) || (efuse_read_index >= size))
        {
            /* can finish now */
            break;
        }
        else
        {
            /* two byte header (byte[7:4] is even and byte[3:0] is 0x0F) */
            if (((efuse_data & 0x0F) == 0x0F) && !(efuse_data & BIT4))
            {
                /*========================*/
                /* two byte header format */
                /*========================*/
                small_section = efuse_data >> 4;

                /* parser 2nd byte of header */
                efuse_one_byte_read(efuse_read_index, &efuse_data);                
                efuse_read_index++;

                if (efuse_read_index >= size)
                {
                    /* no enough space to capture data context, so exit now */
                    break;
                }
                
                large_section = efuse_data >> 4;                
                efuse_header_index = 16 + (small_section >> 1) + 
                                          ((large_section - 2) << 3);
            }
            else
            {
                /*========================*/
                /* one byte header format */
                /*========================*/                
                efuse_header_index = efuse_data >> 4;
            }
            
            word_enable_map = efuse_data & 0xF;            
            word_offset = efuse_header_index << 2;

            //According to word_enable_map to update sram parameters
            for (word_index = 0; word_index < 4; word_index++)
            {                
                if (!(word_enable_map & (1 << word_index)))
                {
                    UINT8 low_byte;
                    UINT8 high_byte;

                    if ((efuse_read_index + 2) > size)
                    {                    
                        /* no enough pair space to capture data context, 
                           so exit now */
                        overflow = 1;
                        break;                        
                    }

                    /* mapped word has been update */
                    efuse_one_byte_read(efuse_read_index++, &low_byte);    
                    efuse_one_byte_read(efuse_read_index++, &high_byte);

                    word_tmp = low_byte | (high_byte << 8);
                    
                    /* skip wroten data field is 0xffff */
                    if (word_tmp != 0xFFFF)
                    {
                        p_efuse[word_offset] = word_tmp;
                    }
                }
                word_offset++;
            }

            if (overflow)
            {
                break;
            }
        }
    }
    //RT_BT_LOG(BLUE,LMP_PAYLOAD_INFO,1,efuse_data[i]);
}

#ifdef _BT_ONLY_
EFUSE_MANAGE_S efuse_manger;

/**
 * Function     : efuse_page0_init
 *
 * Description  : This function is used to initialize efuse settings
 *
 * Parameters   : None.
 *
 * Returns      : None
 */
void efuse_page0_init(void)
{
    UINT32 temp;
   
    /* enable efuse burn grand */
    WR_8BIT_SYSON_IO(0xCF, 0x69);

    /* enable LDOE25 marco block */
    temp = RD_32BIT_SYSON_IO(0x34);
    WR_32BIT_SYSON_IO(0x34, temp | BIT31);

    efuse_manger.init = TRUE;
    efuse_manger.cur_bank = (temp & 0x300) >> 8;
    efuse_manger.cur_status = EFUSE_PG_STATUS_OK;
}

/**
 * Function     : efuse_page0_set_bank
 *
 * Description  : This function is used to set efuse bank 
 *
 * Parameters   : bank : assigned efuse bank
 *
 * Returns      : TRUE is Pass /  FALSE is Fail
 */
UINT8 efuse_page0_set_bank(UINT8 bank)
{
    UINT32 temp1;

    if (bank > 2)
    {
        return FALSE;
    }
    
    if (!efuse_manger.init)
    {
        efuse_page0_init();
    }

    if (efuse_manger.cur_bank != bank)
    {
        temp1 = RD_32BIT_SYSON_IO(0x34);
        WR_32BIT_SYSON_IO(0x34, (temp1 & ~0x300) | (bank << 8));
        efuse_manger.cur_bank = bank;
    }         

    return TRUE;
}

/**
 * Function     : efuse_page0_access
 *
 * Description  : This function is used to access efuse via indirect access 
 *
 * Parameters   : bank : assigned efuse bank
 *                addr : assigned physical address
 *                *data :assigned written data or read data pointer
 *                wr : write or read
 *
 * Returns      : None
 */
void efuse_page0_access(UINT8 bank, UINT16 addr, UINT8 *data, UINT8 wr)
{
    UINT32 temp1;
    UINT32 temp2;
    UINT8 status = EFUSE_PG_STATUS_WRONG_PARAM;
    UINT8 i = 0;

    do
    {
        if (bank == 0)
        {
            if (addr > 0x7f)
            {
                break;
            }
        }
        else
        {
            if (addr > 0x1ff)
            {
                break;
            }        
        } 

        if (efuse_page0_set_bank(bank) == FALSE)
        {
            break;
        } 

        temp1 = (RD_32BIT_SYSON_IO(0x30) & 0x7FF00000) | (addr << 8);
        if (wr)
        {
            temp1 |= (*data | BIT31);
        }       
        WR_32BIT_SYSON_IO(0x30, temp1); 

        status = EFUSE_PG_STATUS_TIMEOUT;

        for (; i < 100; i++)
        {
            temp2 = RD_32BIT_SYSON_IO(0x30);

            if (wr)
            {
                if (!(temp2 & BIT31))
                {
                    status = EFUSE_PG_STATUS_OK;
                    break;                   
                }                  
            }
            else
            {
                if (temp2 & BIT31)
                {
                    *data = temp2 & 0xff;
                    status = EFUSE_PG_STATUS_OK;
                    break;                   
                }                   
            }           
        }
    }
    while (0);

    efuse_manger.cur_status = status;

    if (status != EFUSE_PG_STATUS_OK)
    {
        RT_BT_LOG(BLUE, MSG_ACCESS_EFUSE, 5, wr, bank, addr, *data, i);
    }       
}

/**
 * Function     : efuse_page0_write
 *
 * Description  : This function is used to write efuse via indirect access 
 *
 * Parameters   : bank : assigned efuse bank
 *                addr : assigned physical address
 *                wdata : one byte data for writing
 *
 * Returns      : None
 */
void efuse_page0_write(UINT8 bank, UINT16 addr, UINT8 wdata)
{
    UINT8 data = wdata;
    efuse_page0_access(bank, addr, &data, TRUE);
}

/**
 * Function     : efuse_page0_read
 *
 * Description  : This function is used to read efuse via indirect access 
 *
 * Parameters   : bank : assigned efuse bank
 *                addr : assigned physical address
 *
 * Returns      : one byte data for reading
 */
UINT8 efuse_page0_read(UINT8 bank, UINT16 addr)
{
    UINT8 data;
    efuse_page0_access(bank, addr, &data, FALSE);
    return data;
}

#ifdef EFUSE_TEST_PG
void efuse_pg_test(void)
{
    UINT16 i;
    UINT16 cnt_all = 0;
    UINT16 cnt_err = 0;    
    UINT8 temp;

    /* check bank 0 */
    for (i = 0; i < 128; i++)
    {
        efuse_page0_write(0, i, i);    
    }

    for (i = 0; i < 128; i++)
    {
        temp = efuse_page0_read(0, i);
        if (temp != i)
        {
            cnt_err++;
            RT_BT_LOG(RED,MSG_EFUSE_COMP_ERROR,4, 0, i, i, temp);
        }
        cnt_all++;        
    }

    /* check bank 1 */
    for (i = 0; i < 512; i++)
    {
        efuse_page0_write(1, i, (UINT8)i);    
    }

    for (i = 0; i < 512; i++)
    {
        temp = efuse_page0_read(1, i);
        if (temp != (UINT8)i)
        {
            cnt_err++;            
            RT_BT_LOG(RED,MSG_EFUSE_COMP_ERROR,4, 1, i, (UINT8)i, temp);
        }
        cnt_all++;        
    }

    /* check bank 2 */
    for (i = 0; i < 512; i++)
    {
        efuse_page0_write(2, i, (UINT8)i);    
    }

    for (i = 0; i < 512; i++)
    {
        temp = efuse_page0_read(2, i);
        if (temp != (UINT8)i)
        {
            cnt_err++;            
            RT_BT_LOG(RED,MSG_EFUSE_COMP_ERROR,4, 2, i, (UINT8)i, temp);
        }
        cnt_all++;           
    }

    RT_BT_LOG(YELLOW, MSG_EFUSE_TEST_CHECK, 2, cnt_err, cnt_all);

}
#endif
#endif

/**
 * Function     : efuse_init
 *
 * Description  : This function is used to initialize efuse software variables,
 *                check auto-load fail and parser efuse content
 *
 * Parameters   : None.
 *
 * Returns      : None
 */
void efuse_init(void)
{
#ifdef _BT_ONLY_
    memset(&efuse_manger, 0, sizeof(efuse_manger));

    /* check page 0 08h [21], 1: autoload success, 0: autoload failed */
    if (!(RD_32BIT_SYSON_IO(0x08) & BIT21))
    {
        efuse_manger.auto_load_fail = TRUE;
        return;
    }
#endif
    
    start_update_parameter_from_efuse();
}


