/***************************************************************************
 Copyright (C) Realtek
 ***************************************************************************/
enum { __FILE_NUM__= 150 };

/* ========================= Include File Section ========================= */
#include "DataType.h"
#include "new_io.h"
#include "common_utils.h"
#include "platform.h"
#include "mint_os.h"
#include "bb_driver.h"

#ifdef _FONGPIN_TEST_PI_ACCESS_3RD_MODEM_
#include "lc_internal.h"
#endif

#if defined(_FONGPIN_TEST_TIMER_POLLING_REG_)
#include "gpio.h"
#endif

/* ====================== Macro Declaration Section ======================= */
/* ==================== Structure Declaration Section ===================== */
/* ===================== Variable Declaration Section ===================== */
#if defined(_FONGPIN_TEST_BT_READ_PAGE0_)|| \
    defined(_FONGPIN_TEST_BT_READ_SIE_)|| \
    defined(_FONGPIN_TEST_PI_ACCESS_3RD_MODEM_)|| \
    defined(_FONGPIN_TEST_TIMER_POLLING_REG_)
extern UINT32 g_fongpin_test_timer_counter;
#endif

#if defined (_SUPPORT_INFO_FROM_SYSTEM_ON_) || defined(_BT_ONLY_)
#include "system_on.h"
extern UINT8 g_chip_id;
#endif


#if defined(_BT_ONLY_) || defined(_SUPPORT_COMBO_RW_PAGE0_)
void indirect_access_write_syson_reg(UINT16 offset, UINT32 wdata, UINT8 type)
{
    UINT32 wValue; 
    UINT16 i;

    if (offset & ((1 << type) - 1))
    {
        /* exception case */
        return;
    }

    wValue = offset | (type << 27) | BIT29; 

    if (type == 0)
    {
        wdata <<= ((offset & 0x03) << 3);
    }

    DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* set data port */
    VENDOR_WRITE(VENDOR_INDIRECT_DATA_REG, wdata);

    /* set offset, access type, generate pulse to start hw */
    VENDOR_WRITE(VENDOR_INDIRECT_CTRL_REG, wValue | BIT26);
    VENDOR_WRITE(VENDOR_INDIRECT_CTRL_REG, wValue);

    /* wait hw to idle */
    for (i = 0; i < 20000; i++)
    {
        if (!(VENDOR_READ(VENDOR_INDIRECT_CTRL_REG) & BIT31))
        {
            break;
        }
    }

    MINT_OS_EXIT_CRITICAL();    
}

UINT32 indirect_access_read_syson_reg(UINT16 offset, UINT8 type)
{
    UINT32 wValue;    
    UINT32 rData;
    UINT16 i;

    if (offset & ((1 << type) - 1))
    {
        /* exception case */
        return 0xdeadbeef;
    }
            
    wValue = offset | (type << 27);    
    rData = 0xdeadbeef;    

	DEF_CRITICAL_SECTION_STORAGE;

    MINT_OS_ENTER_CRITICAL();

    /* set offset, access type, generate pulse to start hw */
    VENDOR_WRITE(VENDOR_INDIRECT_CTRL_REG, wValue | BIT26);
    VENDOR_WRITE(VENDOR_INDIRECT_CTRL_REG, wValue);

    /* wait hw to idle */
    for (i = 0; i < 20000; i++)
    {
        if (!(VENDOR_READ(VENDOR_INDIRECT_CTRL_REG) & BIT31))
        {
            /* read data port */    
            rData = VENDOR_READ(VENDOR_INDIRECT_DATA_REG);  
            break;
        }
    }    

    MINT_OS_EXIT_CRITICAL();   

    return rData;
}

void indirect_access_write_two_byte_syson_reg(UINT16 offset, UINT16 wdata)
{
    indirect_access_write_syson_reg(offset, LSB(wdata), 0);
    indirect_access_write_syson_reg(offset + 1, MSB(wdata), 0);
}
    
UINT8 indirect_access_read_one_byte_syson_reg(UINT16 offset)
{
    UINT8 mod = offset & 0x03;

    if (mod)
    {     
        /* non 4 byte alignment offset */
        
        UINT16 aligned = offset & ~0x03;
        return (indirect_access_read_syson_reg(aligned, 2) >> (mod << 3));        
    }
    else
    {
        /* 4 byte alignmnet offset */
        return indirect_access_read_syson_reg(offset, 0);
    }
}

UINT16 indirect_access_read_two_byte_syson_reg(UINT16 offset)
{
    UINT8 mod = offset & 0x03;

    if (mod == 2)
    {     
        /* non 4 byte alignment offset */        
        UINT16 aligned = offset & ~0x03;
        return (indirect_access_read_syson_reg(aligned, 2) >> 16);        
    }
    else if (mod == 0)
    {
        /* 4 byte alignmnet offset */
        return indirect_access_read_syson_reg(offset, 1);
    }
    else
    {
        return 0xdead;
    }
}
#endif

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8821B_TC_

#ifndef _REMOVE_INDIRECT_READ_PAGE0_
void indirect_access_read_syson_test()
    {
    
#ifdef _FONGPIN_TEST_BT_READ_PAGE0_
    
        UINT16 u16offset;
   
        g_fongpin_test_timer_counter++;
        if(g_fongpin_test_timer_counter<500)
            return;
        
        for(u16offset = 0; u16offset<0x20; u16offset++)
            RT_BT_LOG(BLUE,FONGPIN_READ_PAGE0_1BYTE,2, u16offset, RD_8BIT_COMBO_SYSON_IO(u16offset));
            
        for(u16offset = 0; u16offset<0x20; u16offset = u16offset + 2)
           RT_BT_LOG(BLUE,FONGPIN_READ_PAGE0_2BYTE,2, u16offset, RD_16BIT_COMBO_SYSON_IO(u16offset));
        
        for(u16offset = 0; u16offset<0x20; u16offset = u16offset + 4)
            RT_BT_LOG(BLUE,FONGPIN_READ_PAGE0_4BYTE,2, u16offset, RD_32BIT_COMBO_SYSON_IO(u16offset));     
    
        g_fongpin_test_timer_counter = 0;
    
#endif
    
    }


UINT8 safe_indirect_read_combo_byte_syson_reg(UINT16 offset)
{


/* read system on page 0 */
/* 8761 bt only : operated in 4-byte aligned */
/* 8821b combo: operated in non 4-byte aligned */
/* this function only for 8703b and later */

    UINT32 rData;
    UINT32 rData2=0;
    UINT8 loop=0;

    while(loop<10)
    {
        rData = indirect_read_combo_byte_syson_reg(offset);
        if(loop == 0)
        {
            rData2 = rData;
            loop++;
            continue;
        }
        else
        {
            //rData2 = rData;
            if((rData2 == rData)/*&&(loop>=1)*/)
            {
                break;
            }
            else
            {
                rData2 = rData;
            }
        }
        loop++;
        
        if(loop == 10)
        {
            RT_BT_LOG(RED, SAFE_READ_PAGE0_ERROR_MSG, 0, 0);
            break;
        }


    };

return rData;


}

UINT8 indirect_read_combo_byte_syson_reg(UINT16 offset)
{
    /* read system on page 0 */
    /* 8761 bt only : operated in 4-byte aligned */
    /* 8821b combo: operated in non 4-byte aligned */
    /* this function only for 8703b and later */

    UINT32 rData;
    UINT8 u8offset_L;
    UINT8 u8offset_H;    
    //UINT8 u8ReadTmpH, u8ReadTmpL;

    u8offset_H =  (UINT8)((offset & 0x0300)>>8);
    u8offset_L = (UINT8)(offset & 0x00FF);

    //DEF_CRITICAL_SECTION_STORAGE;
    //MINT_OS_ENTER_CRITICAL();   
    VENDOR_BYTE_WRITE(BTON_INDIRECT_READ_PAGE0_REGISTER + 2, u8offset_H);
    VENDOR_BYTE_WRITE(BTON_INDIRECT_READ_PAGE0_REGISTER + 1, u8offset_L);
    rData = VENDOR_BYTE_READ(BTON_INDIRECT_READ_PAGE0_REGISTER);
    //MINT_OS_EXIT_CRITICAL();   

    return rData;

  
}



UINT16 indirect_read_combo_word_syson_reg(UINT16 offset)
{
    /* read system on page 0 */
    /* 8761 bt only : operated in 4-byte aligned */
    /* 8821b combo: operated in non 4-byte aligned */
    /* this function only for 8703b and later */

    UINT16 u16Hbyte;
    UINT16 u16LByte;
    
    /* there is no 4 byte alignment issue */
    u16Hbyte =  safe_indirect_read_combo_byte_syson_reg(offset+1);
    u16LByte =  safe_indirect_read_combo_byte_syson_reg(offset);
    return (u16Hbyte<<8)|u16LByte;
}


UINT32 indirect_read_combo_dword_syson_reg(UINT16 offset)
{
    /* read system on page 0 */
    /* 8761 bt only : operated in 4-byte aligned */
    /* 8821b combo: operated in non 4-byte aligned */
    /* this function only for 8703b and later */
   
    UINT32 u32Byte0, u32Byte1, u32Byte2, u32Byte3;

    u32Byte3 =  safe_indirect_read_combo_byte_syson_reg(offset+3);
    u32Byte2 =  safe_indirect_read_combo_byte_syson_reg(offset+2);
    u32Byte1 =  safe_indirect_read_combo_byte_syson_reg(offset+1);
    u32Byte0 =  safe_indirect_read_combo_byte_syson_reg(offset);
    return (u32Byte3<<24)|(u32Byte2<<16)|(u32Byte1<<8)|u32Byte0;

}
#endif
UINT16 indirect_access_read_32k_reg(UINT16 offset)
{
    UINT16 read_data;

#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_
    PON_F00_REG_S_TYPE pon_f00;

    pon_f00.d32 = VENDOR_READ(PON_F00_REG);
	pon_f00.ind_32k_rw = 0;
	pon_f00.ind_32k_addr = offset;
	VENDOR_WRITE(PON_F00_REG, pon_f00.d32);
	
    pon_f00.d32 = VENDOR_READ(PON_F00_REG);
	read_data = (UINT16) pon_f00.ind_32k_wdata;
#else
    BTON_5C_REG_S_TYPE bton_5c;

    bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
	bton_5c.ind_32k_rw = 0;
	bton_5c.ind_32k_addr = offset;
	VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);
	
    bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
	read_data = (UINT16) bton_5c.ind_32k_wdata;
#endif	
    return read_data;
}

void indirect_access_write_32k_reg(UINT16 offset, UINT16 wdata)
{
#ifdef _NEW_BTON_DESIGN_AFTER_RTL8703B_TC_
    PON_F00_REG_S_TYPE pon_f00;

    pon_f00.d32 = VENDOR_READ(PON_F00_REG);
	pon_f00.ind_32k_rw = 0;    
	pon_f00.ind_32k_addr = offset;
	pon_f00.ind_32k_wdata = wdata;	
	VENDOR_WRITE(PON_F00_REG, pon_f00.d32);
    
	pon_f00.ind_32k_rw = 1;
	VENDOR_WRITE(PON_F00_REG, pon_f00.d32);
    
#else		
    BTON_5C_REG_S_TYPE bton_5c;

    bton_5c.d32 = VENDOR_READ(BTON_5C_REG);
	bton_5c.ind_32k_rw = 1;
	bton_5c.ind_32k_addr = offset;
	bton_5c.ind_32k_wdata = wdata;	
	VENDOR_WRITE(BTON_5C_REG, bton_5c.d32);
#endif	
}

#endif

#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
UINT16 safe_indirect_read_sie(UINT8 type, UINT8 u8wAddr)
{
    //UINT32 Sie_wAddr;
    UINT16 Sie_rData = 0xFFFF;
    UINT16 Sie_rData2 = 0x0000;
    UINT8 loop = 0;
    //UINT8 theSame = 0;
    
    if(((type != READ_SIE_BYTE)&&(type != READ_SIE_WORD))\
        ||((type == READ_SIE_WORD)&&((u8wAddr & 0x01)!=0)))
    {   // error case
        RT_BT_LOG(RED, READ_SIE_ERROR_MSG, 0, 0);
        return Sie_rData;
        //return 0xFFFF;
    }        

    while(loop<10)
    {
        Sie_rData = indirect_read_sie(type, u8wAddr);
        if(loop ==0)
        {
            Sie_rData2 = Sie_rData;
            loop++;
            continue;    
        }
        else
        {

            if((Sie_rData2 == Sie_rData)/*&&(loop>=1)*/)
            {
                break;
            }
            else
            {
                Sie_rData2 = Sie_rData;
            }

        }
        loop++;
        
        if(loop == 10)
        {
            RT_BT_LOG(RED, SAFE_READ_SIE_ERROR_MSG, 0, 0);
            break;
        }


    };

    return Sie_rData;
}
#endif


#ifdef _SUPPORT_FW_INDIRECT_READ_SIE_
UINT16 indirect_read_sie(UINT8 type, UINT8 u8wAddr)
{
    // after 8703b, bt fw can read sie register
    // type: 0, return byte data, 1, return word data
    // wAddress [23:16]: word address
    // rdata [15:0]

    // example
    // for 8821b, if input addr = 0x80, 0x81, you will get the same data
    // for 8703b and 8822b, 0x81 is illegal addr, should be word addr  
    

    UINT32 Sie_wAddr;
    UINT16 Sie_rData = 0xFFFF;
    
    if(((type != READ_SIE_BYTE)&&(type != READ_SIE_WORD))\
        ||((type == READ_SIE_WORD)&&((u8wAddr & 0x01)!=0)))
    {   // error case
        //RT_BT_LOG(RED, READ_SIE_ERROR_MSG, 0, 0);
        return Sie_rData;
        //return 0xFFFF;
    }        

    if(type == READ_SIE_BYTE)
    {
        // read byte data


#if !defined(_SUPPORT_HW_READ_SIE_BYTE_ADDR_)&& !defined(_RTL8723D_SPECIFIC_)        
        Sie_wAddr = ((u8wAddr>>1))<<16; // change to word addr, // 8703b
        
#else
        Sie_wAddr = ((u8wAddr>>1)<<1)<<16; // Keep byte addr, // 8723d, 
#endif
       
        VENDOR_WRITE(VENOR_REG_READ_SIE, Sie_wAddr);
        Sie_rData = (UINT16)(VENDOR_READ(VENOR_REG_READ_SIE));
        if((u8wAddr & 0x01)==0)
            return (Sie_rData&0x00FF);//return low byte
        else
            return (Sie_rData>>8); //return high byte
    }
    else
    {
     
        //read word data         
#if !defined(_SUPPORT_HW_READ_SIE_BYTE_ADDR_)&& !defined(_RTL8723D_SPECIFIC_)        
                Sie_wAddr = ((u8wAddr>>1))<<16; // change to word addr, // 8703b
#else
                Sie_wAddr = ((u8wAddr>>1)<<1)<<16; // Keep byte addr, // 8723d, 
#endif


        VENDOR_WRITE(VENOR_REG_READ_SIE, Sie_wAddr);
        
        if(g_chip_id == CHIP_ID_8822B)
        {
            UINT8 Idx;
            for(Idx = 0; Idx<4; Idx++)
            {
                rlx4081NopDelay();
            }
            //rlx4081NopDelay();
            //rlx4081NopDelay();
            //rlx4081NopDelay();
        }

        Sie_rData = (UINT16)(VENDOR_READ(VENOR_REG_READ_SIE));
    }
    return Sie_rData;
}

void indirect_access_read_sie_test()
{
#if defined(_FONGPIN_TEST_BT_READ_SIE_)&&defined(_SUPPORT_FW_INDIRECT_READ_SIE_)
    UINT32 addr;
    UINT16 Sie_rData;
    UINT8 u8String1[9] = {0x09, 0x03, 0x52, 0x65, 0x61, 0x6C, 0x74, 0x65, 0x6B};//0xFE80~0xFE88
    UINT16 u16String[5] = {0x0309, 0x6552, 0x6C61, 0x6574, 0x006B};
    UINT8 u8String1_1[23] = {0x00};//0xFE89~0xFE9F
    UINT8 u8String2[23] = {0x17, 0x03, 0x38, 0x30, 0x32, 0x2E, 0x31, 0x31, 0x61, 0x63, 0x20, 0x57, 0x4C, 0x41, 0x4E, 0x20, 0x41, 0x64, 0x61, 0x70, 0x74, 0x65, 0x72};//0xFEA0~0xFEB6
    UINT8 u8String2_1[25] = {0x00};//0xFEB7~0xFECF
    UINT8 u8String3[14] = {0x0E, 0x03, 0x30, 0x30, 0x65, 0x30, 0x34, 0x63, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31};//0xFED0~0xFEDF
    UINT8 i;
    
    // check sie register
    // check type 0   
    g_fongpin_test_timer_counter++;
    if(g_fongpin_test_timer_counter<500)
        return;
    
    for( addr = 0x80; addr<=0x88; addr = addr + 1)
    {
        Sie_rData = safe_indirect_read_sie(0, addr);
        if(Sie_rData != u8String1[addr-0x80])
        {
            for( i = 0; i<3; i++)
                RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }
    
    for( addr = 0x89; addr<=0x9F; addr = addr + 1)
    {
        Sie_rData = safe_indirect_read_sie(0, addr);
        if(Sie_rData != u8String1_1[addr-0x89])
        {
            for( i = 0; i<3; i++)
                RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }

    for( addr = 0xA0; addr<=0xB6; addr = addr + 1)
    {
        Sie_rData = safe_indirect_read_sie(0, addr);
        if(Sie_rData != u8String2[addr-0xA0])
        {
            for( i = 0; i<3; i++)
                RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }

    for( addr = 0xB7; addr<=0xCF; addr = addr + 1)
    {
        Sie_rData = safe_indirect_read_sie(0, addr);
        if(Sie_rData != u8String2_1[addr-0xB7])
        {
            for( i = 0; i<3; i++)
               RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }

    for( addr = 0xD0; addr<=0xDD; addr = addr + 1)
    {
        Sie_rData = safe_indirect_read_sie(0, addr);
        if(Sie_rData != u8String3[addr-0xD0])
        {
            for( i = 0; i<3; i++)
                RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }


    // check type 1, address ok
    for( addr = 0x80; addr<=0x88; addr = addr + 2)
    {
        Sie_rData = safe_indirect_read_sie(1, addr);
        if(Sie_rData != u16String[(addr-0x80)/2])
        {
            for( i = 0; i<3; i++)
                RT_BT_LOG(RED, READ_SIE_DATA, 2,addr, Sie_rData);
        }
    }
    
    // check type1, illegal address
    Sie_rData = safe_indirect_read_sie(1, 0x89);
    Sie_rData = safe_indirect_read_sie(1, 0x8b);

    g_fongpin_test_timer_counter = 0;
  
#endif
}


#endif




#if defined(_SUPPORT_COMBO_RW_SIE_BY_PAGE0_) || defined(_BT_ONLY_)
UINT8 read_sie_by_page0(UINT8 u8Offset)
{
    // valid after 8723d & 8821C
    
    UINT8 u8Sie_rData;// = 0xFF;
    
    PAGE0_REG_0xE0_SIE_CTRL_S page0_reg;
    page0_reg.d32 = RD_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL);
    page0_reg.b.sie_address = u8Offset;
    page0_reg.b.sie_bypass_ioreg_if = 1;
    WR_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL, page0_reg.d32);

    //page0_reg.b.sie_bypass_ioreg_if = 1;
    page0_reg.d32 = RD_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL);
    u8Sie_rData = page0_reg.b.read_data;

    //page0_reg.d32 = RD_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL);
    page0_reg.b.sie_bypass_ioreg_if = 0;
    WR_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL, page0_reg.d32);
    
    return u8Sie_rData;

}


void write_sie_by_page0(UINT8 u8Offset, UINT8 u8Value)
{

    PAGE0_REG_0xE0_SIE_CTRL_S page0_reg;
    page0_reg.d32 = RD_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL);
    //page0_reg.b.sie_address = u8Offset;
    page0_reg.b.sie_bypass_ioreg_if = 1;
    //page0_reg.b.sie_write_en = 1;
    WR_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL, page0_reg.d32);

    page0_reg.d32 = RD_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL);
    
    
    page0_reg.b.sie_address = u8Offset;
    page0_reg.b.write_data = u8Value;
    page0_reg.b.sie_bypass_ioreg_if = 1;
    page0_reg.b.sie_write_en = 1;
    WR_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL, page0_reg.d32);

    // park sie
    page0_reg.b.sie_bypass_ioreg_if = 0;
    page0_reg.b.sie_write_en = 0;
    WR_32BIT_SYSON_IO(PAGE0_REG_SIE_CTRL, page0_reg.d32);    
}



#endif






#ifdef _FONGPIN_TEST_PI_ACCESS_3RD_MODEM_
UINT16 MODEM_REG[25] = {0, 0x4155, 0x94DB, 0x3063, 0x311E,\
                        0xB0, 0x3322, 0x1333, 0x569C, 0xF48,\
                        0x0, 0x5A9E, 0x690F, 0x0, 0x37F2, 0x9671,\
                        0xF98E, 0x56E7, 0x5E72, 0x7334, 0x58CC, 0x475C,\
                        0x400, 0xEC0, 0x7C3B};


void ThirdGenModemPiAccessTest()
{
    UINT32 u32Value_SI, u32Value_PI;
    UINT32 PI_Write_Addr;
    UINT32 addr;
    UINT8 u8CompareFlag = 1 ;







    
  
        g_fongpin_test_timer_counter ++;
    
        if(g_fongpin_test_timer_counter<500)
            return;
#if 1          
        //---------------- 
        // 27~22 page
        // 21~16 offset
        // addr = page|offset
        u32Value_PI = RTK_READ_MODEM_REG_PI(0, 0x7);// should be 0x0333
        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, 0x07, u32Value_PI);
        u32Value_PI = RTK_READ_MODEM_REG_PI(1, 0x1a);// should be 0xfff5
        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, 0x1a, u32Value_PI);
        u32Value_PI = RTK_READ_MODEM_REG_PI(2, 0x01);// should be 0x0087
        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, 0x01, u32Value_PI);
        u32Value_PI = RTK_READ_MODEM_REG_PI(3, 0x01);// should be 0x3322
        RT_BT_LOG(YELLOW, YL_DBG_HEX_2, 2, 0x01, u32Value_PI);

        // SI read
        for( addr = 0; addr<50; addr = addr + 2)
        {
            u32Value_SI = rtk_read_modem_radio_reg((UCHAR)TRANS_MODEM_REG(addr), TYPE_MODEM);
            RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2, addr, u32Value_SI);
            
        }

        // test PI read
        for( addr = 0; addr<25; addr = addr + 1)
        {
            u32Value_PI = RTK_READ_MODEM_REG_PI(0, addr);
            if(MODEM_REG[addr] != u32Value_PI)
            {
               u8CompareFlag = 0;
               RT_BT_LOG(RED, YL_DBG_DEC_1, 3, addr, u32Value_PI, MODEM_REG[addr]);
            }
            
        }
        
        if( u8CompareFlag == 1 ) // step 1 pass
            RT_BT_LOG(GREEN, YL_DBG_DEC_1, 1, 0x1);
            
        // test PI write, set data to 0xFFFF
        PI_Write_Addr = 0;
        for( addr = 0; addr<25; addr = addr + 1)
        {
            RTK_WRITE_MODEM_REG_PI(0, addr, 0xFFFF);

        }

        // check is 0xFFFF or not
        for( addr = 0; addr<25; addr = addr + 1)
        {
            u32Value_PI = RTK_READ_MODEM_REG_PI(0, addr);

            if(0xFFFF != u32Value_PI)
            {
                u8CompareFlag = 0;
                RT_BT_LOG(RED, YL_DBG_DEC_1, 3, addr, u32Value_PI, MODEM_REG[addr]);
            }
            
          
         }  
        
        if( u8CompareFlag == 1 ) // step 2 pass
            RT_BT_LOG(GREEN, YL_DBG_DEC_1, 1, 0x2);  

        for( addr = 0; addr<25; addr = addr + 1)
        {
            RTK_WRITE_MODEM_REG_PI(0, addr, MODEM_REG[addr]);
        }


        RT_BT_LOG(GREEN, YL_DBG_DEC_1, 1, 0x3); 
        
        g_fongpin_test_timer_counter = 0;
#endif    
    
}

#endif

#if defined(_FONGPIN_TEST_TIMER_POLLING_REG_)
void timer_polling_io_dbg()
{
    g_fongpin_test_timer_counter ++;
    
    if(g_fongpin_test_timer_counter<500)
        return;

    PARK_PCM_GPIO_PIN_S_TYPE efuse_park_pcm_mux_state;
    *(UINT16*)&efuse_park_pcm_mux_state = otp_str_data.efuse_park_pcm_mux_mgr;
    RT_BT_LOG(BLUE, YL_DBG_HEX_2, 2,efuse_park_pcm_mux_state.d16, efuse_park_pcm_mux_state.b.restore_pcm_mux_state_sel);

    RT_BT_LOG(WHITE, YL_DBG_HEX_3, 3,VENDOR_READ(REG_BTON_REG_CTRL0), VENDOR_READ(REG_BTON_REG_CTRL1), VENDOR_READ(REG_VEN_GPIO_CTRL));


 
    g_fongpin_test_timer_counter = 0;

}
#endif


