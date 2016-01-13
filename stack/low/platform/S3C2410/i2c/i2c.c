/***************************************************************************
 Copyright (C) RealTek Ltd.
 This module is a confidential and proprietary property of RealTek and
 a possession or use of this module requires written permission of RealTek.
 ***************************************************************************/

enum { __FILE_NUM__= 216 };

#include "I2C.h"
#include "platform.h"
#include "power_control.h" // for i2c struct
#include "bb_driver.h"
#include "UartPrintf.h"
#include "logger.h"
#include "DataType.h"

#ifdef _SUPPORT_8723D_VENDOR_I2C_
#include "gpio.h"
#endif

#ifdef _SUPPORT_8822B_I2C_

#ifdef _ROM_CODE_PATCHED_
//PF_ROM_CODE_PATCH_FUNC rcp_i2c_func = NULL;
#endif

//2 I2C:
//2 read from page0
//2 write to bton


UINT8 i2c_delay;

void i2c_init()
{
    // get mode from efuse?
    BTON_I2C_REG_S_TYPE reg_I2c;
    PAGE0_NFC_PAD_CONTROL_S page0_i2c_reg;
    UINT8 u8mode = I2C_STARTARD;
    
    page0_i2c_reg.d32 = I2C_IO_READ();
    // check if the nfc io bt controlable?
    if(page0_i2c_reg.b.nfc_pad_control_sel != BT_CTRL)
    {
        RT_BT_LOG(RED, I2C_I2C_INIT_FAIL, 1,page0_i2c_reg.d32);
        return;
    }

    // ok, we can control the nfc pad
    i2c_mode_sel(u8mode);

   
    RT_BT_LOG(WHITE, I2C_I2C_INIT_DONE, 0,0);
    
}/*end of i2c_init*/
void i2c_data_write(UINT32 u32length)
{
    // write
    // 1. master address slave
    // 2. master send data
    // 3. master termate the transfer
}

void i2c_data_read(UINT32 u32length)
{
    // read
    // 1. master address slave
    // 2. master read data from slave
    // 3. master termate the transfer

    //I2C_CTRL_0x51_BYTE_ACCESS bton_byte_reg_0x51;
    BTON_I2C_REG_S_TYPE page0_0x10A8_reg;    
    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    //bton_byte_reg_0x51.d8 = I2C_CTRL_READ();



    
    page0_0x10A8_reg.d32 = I2C_IO_READ();



    
}

/*
UINT32 r_sym_nfc_data_o             :1; // [2] 
UINT32 r_sym_nfc_clk_o              :1; // [3] 
UINT32 r_sym_nfc_rfdis_o            :1; // [4] 
UINT32 r_sym_nfc_int_o              :1; // [5]  
UINT32 r_sym_nfc_data_oe            :1; // [6] 
UINT32 r_sym_nfc_clk_oe             :1; // [7] 
UINT32 r_sym_nfc_rfdis_oe           :1; // [8] 
UINT32 r_sym_nfc_int_oe             :1; // [9]
UINT32 r_sym_nfc_pad_pull_en        :1; //[10]
UINT32 r_sym_nfc_data_pull          :1; //[11]
UINT32 r_sym_nfc_clk_pull           :1; //[12]
UINT32 r_sym_nfc_pad_shutdown       :1; //[13] 


*/


void i2c_toggle_scl_sda(UINT32 bitCount, UINT8* pdata)
{
    UINT32 loop;
    BTON_I2C_REG_S_TYPE reg_I2c;
    UINT8 u8bitdata;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);



    
    // CLK LOW->CLK HIGH
    for(loop = 1; loop<=bitCount; loop++)
    {
        //pf_delay_us(tSU;DAT);
        //byteIndex = bitCount/8;
        u8bitdata = (((*pdata)>>(bitCount-loop))&0x01);
        //RT_BT_LOG(color, log_str_index, para_num,...);
        // clock low duty
        reg_I2c.r_sym_nfc_clk_oe = 1;
        reg_I2c.r_sym_nfc_data_oe = 1;
        reg_I2c.r_sym_nfc_clk_o = 0;
        reg_I2c.r_sym_nfc_data_o = u8bitdata;//(*pdata+byteIndex);
        VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
        pf_delay_us(i2c_delay);
        //RT_BT_LOG(WHITE, YL_DBG_HEX_2, 2,*pdata,u8bitdata, );

        // clock high duty
        reg_I2c.r_sym_nfc_clk_oe = 1;
        reg_I2c.r_sym_nfc_data_oe = 1;        
        reg_I2c.r_sym_nfc_clk_o = 1;
        reg_I2c.r_sym_nfc_data_o = u8bitdata;
        VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER,reg_I2c.d32);
        pf_delay_us(i2c_delay);        
    }

}

void i2c_gen_rw_flag(UINT8 rw)
{
    UINT8 data = rw;
    i2c_toggle_scl_sda(1, &data);
}

void i2c_read_byte_data(UINT8* pdata)
{

    PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;    
    UINT8 bitIdx;
    //UINT32 byteIndex;
    //UINT32 bitCount = ByteCount<<3;

    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    *pdata = 0;


    // CLK LOW->CLK HIGH
    for(bitIdx = 7; 0<=bitIdx; bitIdx--)
    {
        //byteIndex = bitCount/8;

        reg_I2c.r_sym_nfc_clk_oe = 1;
        reg_I2c.r_sym_nfc_data_oe = 0;
        reg_I2c.r_sym_nfc_clk_o = 0;
        VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER,reg_I2c.d32);
        pf_delay_us(i2c_delay);

        
        reg_I2c.r_sym_nfc_clk_oe = 1;
        reg_I2c.r_sym_nfc_data_oe = 0;
        reg_I2c.r_sym_nfc_clk_o = 1;
        VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
        pf_delay_us(i2c_delay);
        
        page0_0x10A8_reg.d32 = I2C_IO_READ();
        *pdata |= page0_0x10A8_reg.b.nfc_pad_data_in<<bitIdx;
        if(bitIdx>7)
            break;
        //RT_BT_LOG(GREEN, YL_DBG_DEC_1, 1,*pdata);
    }
    //RT_BT_LOG(GREEN, YL_DBG_DEC_1, 1,*pdata);
    
}

void i2c_write_byte(UINT8 data)
{
   i2c_toggle_scl_sda(8,&data);
    
}

void i2c_mode_sel(UINT8 mode)
{
//enum MODE{STARTARD=0, FAST, HS};//100kbps, 400kbps, 3.4Mbps

    switch(mode)
    {
        case I2C_STARTARD:
            i2c_delay = 10;
        break;
        case I2C_FAST:
            i2c_delay = 2.5;
        break;
        case I2C_HS: //?????????????????
            // error case
            i2c_delay = 1;
        break;

        default:
        break;
    };

    
        
}
void i2c_output_io(UINT8 u8ioSel, UINT8 u8Level)
{
    
}
BOOLEAN check_i2c_idle(UINT8 loop)
{
    //loop = 0;
    BOOLEAN result = FALSE;
    UINT8 idx;
    for(idx = 0; idx<loop; idx++)
    {
        if(!i2c_is_bus_idle())
        {
            //RT_BT_LOG(WHITE, log_str_index, para_num,...);
            //return;
            //continue;
            pf_delay(1);
            result = FALSE;
        }
        else
        {
            //break;
            result = TRUE;
            break;
        }

    }

    return result;

}
void i2c_gen_start_condition_S()
{
    // start: SCL = 1, SDA = 1->0
    //PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;    
    BTON_I2C_REG_S_TYPE reg_I2c;
    //UINT8 loop;
    // you shoudl check bus is idle or not
    // implement this code in multi master

        
    
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);
    //page0_0x10A8_reg.d32 = I2C_IO_READ();    

/*
    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 0;
    //reg_I2c.r_sym_nfc_data_o = 1;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
*/




    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 1;
    //reg_I2c.r_sym_nfc_data_o = 1;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);

    // set SCL = 1, and SDA = 0
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;    
    reg_I2c.r_sym_nfc_clk_o = 1;
    reg_I2c.r_sym_nfc_data_o = 0;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
       

  
}
void i2c_gen_start_condition_S1()
{
    // start: SCL = 1, SDA = 1->0
    PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;    
    BTON_I2C_REG_S_TYPE reg_I2c;
    //UINT8 loop;
    // you shoudl check bus is idle or not
    // implement this code in multi master

        
    
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);
    page0_0x10A8_reg.d32 = I2C_IO_READ();    

/*
    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 0;
    reg_I2c.r_sym_nfc_data_o = 1;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);

*/



    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 0;
    //reg_I2c.r_sym_nfc_data_o = 1;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);


    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 1;
    //reg_I2c.r_sym_nfc_data_o = 1;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);


    // set SCL = 1, and SDA = 0
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;    
    reg_I2c.r_sym_nfc_clk_o = 1;
    reg_I2c.r_sym_nfc_data_o = 0;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
       

  
}

void i2c_gen_stop_condition_P()
{


    BTON_I2C_REG_S_TYPE reg_I2c;
     reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);



    
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 0;
    reg_I2c.r_sym_nfc_data_o = 0;
    //I2C_CTRL_WRITE(reg_I2c.d32);
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);





    // stop: SCL = 1, SDA = 0->1

 
/*
    // you shoudl check bus is idle or not
    // implement this code in multi master
    if(!check_i2c_idle(500))
        {
            RT_BT_LOG(WHITE, I2C_IS_BUS_BUSY_TIMEOUT, 0, 0);
            return;
        }
*/






    // set SCL = 1,
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 1;
    reg_I2c.r_sym_nfc_data_o = 0;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);

    // set SCL = 1, and SDA = 0
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;    
    reg_I2c.r_sym_nfc_clk_o = 1;
    //reg_I2c.r_sym_nfc_data_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);

}


void i2c_gen_start_byte_Sr()
{
    // start byte = 00000001
    //I2C_CTRL_0x51_BYTE_ACCESS bton_byte_reg_0x51;
    //bton_byte_reg_0x51.d8 = I2C_REG_READ(bton_0x51_i2c);
   
}

void i2c_byte_data_out(UINT8 data, UINT8 length)
{
    //I2C_CTRL_0x51_BYTE_ACCESS bton_byte_reg_0x51;
    //bton_byte_reg_0x51.d8 = I2C_REG_READ(bton_0x51_i2c);

}


void i2c_address_slave(UINT8 addr)
{
    // before send data, it master should address the slave first
    // 1. address is 7 bits
    // 2. address follow the START
    // note: new spec: 10 bits

    //UINT8  = 0x00;
    i2c_toggle_scl_sda(8,&addr);


    

}

BOOLEAN i2c_is_bus_idle()
{
    // it can use the i2c bus if the bus is idle
    // the i2c bus is wired-AND connection


    PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;
    
    i2c_release_bus();
    
    page0_0x10A8_reg.d32 = I2C_IO_READ();
    //RT_BT_LOG(WHITE, I2C_IS_BUS_IDLE, 1, page0_0x10A8_reg.d32);

    if((page0_0x10A8_reg.b.nfc_pad_clk_in != 1)||(page0_0x10A8_reg.b.nfc_pad_data_in != 1))
    //if(page0_0x10A8_reg.b.nfc_pad_clk_in != I2C_IDLE)
    {

        return FALSE;
    }
    //RT_BT_LOG(WHITE, I2C_BUS_IDLE, 1,page0_0x10A8_reg.d32);
    return TRUE;
}


void i2c_check_slave_force_wait()
{
    // check if scl keep 0
    // spec. 7.1
}

BOOLEAN i2c_is_slave_nak()
{
    // 1. gen scl pulse and read sda

    PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;
    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    // the master must generate an extra clock pulse to get ack bit
    // set
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 0;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 0;
    reg_I2c.r_sym_nfc_clk_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    page0_0x10A8_reg.d32 = I2C_IO_READ();

    return page0_0x10A8_reg.b.nfc_pad_data_in;

    /*
    if(page0_0x10A8_reg.b.r_nfc_data_pad_i == I2C_ACK)
    {   // ack
        return I2C_ACK;

        
    }
    else
    {   // nak
        // slave is busy.
        // master can generate repeated STARTs or a STOP

    }
    */

}

void i2c_master_gen_handshake(BOOLEAN bHandshake)
{
    // 1. gen scl pulse and read sda

    //PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;
    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    // the master must generate an extra clock pulse to get ack bit
    // set
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 0;
    //reg_I2c.r_sym_nfc_data_o = 1;
    reg_I2c.r_sym_nfc_data_o = bHandshake;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 0;
    reg_I2c.r_sym_nfc_data_o = bHandshake;
    //reg_I2c.r_sym_nfc_data_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    //page0_0x10A8_reg.d32 = I2C_IO_READ();

    //return page0_0x10A8_reg.b.nfc_pad_data_in;    
}

/*
void i2c_master_gen_nak()
{
    // 1. gen scl pulse and read sda

    //PAGE0_NFC_PAD_CONTROL_S page0_0x10A8_reg;
    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    // the master must generate an extra clock pulse to get ack bit
    // set
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 0;
    reg_I2c.r_sym_nfc_data_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    
    reg_I2c.r_sym_nfc_clk_oe = 1;
    reg_I2c.r_sym_nfc_data_oe = 1;
    reg_I2c.r_sym_nfc_clk_o = 0;
    reg_I2c.r_sym_nfc_data_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    pf_delay_us(i2c_delay);
    //page0_0x10A8_reg.d32 = I2C_IO_READ();

    //return page0_0x10A8_reg.b.nfc_pad_data_in;    
}
*/



void i2c_release_bus()
{
    BTON_I2C_REG_S_TYPE reg_I2c;
 
    reg_I2c.d32 = VENDOR_READ(BTON_I2C_CONTROL_REGISTER);

    reg_I2c.r_sym_nfc_clk_oe = 0;
    reg_I2c.r_sym_nfc_data_oe = 0;
    //reg_I2c.r_sym_nfc_clk_o = 0;
    //reg_I2c.r_sym_nfc_data_o = 1;
    VENDOR_WRITE(BTON_I2C_CONTROL_REGISTER, reg_I2c.d32);
    //pf_delay_us(i2c_delay);    
}

void i2c_interrupt_handler()
{
/*    
    BTON_SCOREBOARD_REG_S_TYPE reg_coreboard;
 
    reg_coreboard.d32 = VENDOR_READ(BTON_DLPS_CONTROL_REGISTER);
    
#ifdef _FONGPIN_TEST_SCOREBOARD_LOG_    
    RT_BT_LOG(BLUE, SCOREBOARD_REG, 1, reg_coreboard.d32);
#endif

#ifdef _ROM_CODE_PATCHED_
        if (rcp_scoreboard_intrhandler_func != NULL)
        {
            if(rcp_scoreboard_intrhandler_func((void*)&reg_coreboard.d32))
            {
                return;
            }
        }    
#endif
    
    //clear interrupt
    SAFE_CLEAR_SCOREBOARD_INT;
*/
}/*end of i2c_interrupt_handler*/



#ifdef _FONGPIN_TEST_I2C_RW_EERPOM_
void i2c_gen_24LC64_control()
{
    //control = 7bit = 1010(control)000(address)
    UINT8 data = 0x50;
    i2c_toggle_scl_sda(7,&data);
}

//2 test case 1
void i2c_test_24LC64_current_address_read()
{
    UINT8 loopcnt = 0;
    UINT8 u8Read;

    // start bit, control byte, 
    while(1)
    {
        i2c_gen_start_condition_S();
        
        i2c_gen_24LC64_control();
        
        i2c_gen_rw_flag(I2C_READ);
        if(!i2c_is_slave_nak()||(loopcnt>5))
        {
            break;
        }
        loopcnt++;
       
    }

    // get data
    i2c_read_byte_data(&u8Read);
    if(i2c_is_slave_nak())
    {
        i2c_gen_stop_condition_P();
    }
    else
    {
        // nothing in the datasheet
        //i dont know;
    }
    RT_BT_LOG(WHITE, I2C_READ_ADDRESS, 1,u8Read);
}
//2 test case 2
void i2c_test_24LC64_random_read()
{
    UINT8 loopcnt = 0;
    UINT8 u8Read;

    // start bit, control byte, 
    while(1)
    {
        i2c_gen_start_condition_S();
        i2c_gen_24LC64_control();
        i2c_gen_rw_flag(I2C_WRITE);// write address
        if(!i2c_is_slave_nak())
        {
            break;
        }
        
        /*
        if(!i2c_is_slave_nak()||(loopcnt>5))
        {
            break;
        }
        loopcnt++;
       */
    }
#if 1    
    // address
    // addr = 10bit valid
    UINT16 u16Addr = 0x0000;    
    i2c_address_slave(((u16Addr)>>8));
    while(1)
    {
        if(!i2c_is_slave_nak())
            break;
    }


    i2c_address_slave((u16Addr));
    while(1)
    {
        if(!i2c_is_slave_nak())
            break;
    }
#endif    

    loopcnt = 0;
    while(1)
    {
        i2c_gen_start_condition_S1();
        i2c_gen_24LC64_control();
        i2c_gen_rw_flag(I2C_READ);
        if(!i2c_is_slave_nak()||(loopcnt>5))
        {
            break;
        }
        loopcnt++;
       
    }
#if 1


    
    // get data
    i2c_read_byte_data(&u8Read);
    //i2c_master_gen_nak();
    i2c_master_gen_handshake(I2C_NAK);
    i2c_gen_stop_condition_P();
/*
    if(i2c_is_slave_nak())
    {
        i2c_gen_stop_condition_P();
    }
    else
    {
        // nothing in the datasheet
        //i dont know;
    }
*/
    RT_BT_LOG(WHITE, I2C_READ_DATA, 2,u16Addr,u8Read);
#endif
}


//2 test case 3
void i2c_test_24LC64_byte_write()
{
    UINT8 loopcnt = 0;
    UINT8 u8write;

    // start bit, control byte, 
    while(1)
    {
        i2c_gen_start_condition_S();
        i2c_gen_24LC64_control();
        i2c_gen_rw_flag(I2C_WRITE);// write address
        if(!i2c_is_slave_nak()||(loopcnt>5))
        {
            break;
        }
        loopcnt++;
       
    }
    // address
    // addr = 10bit valid
    UINT16 u16Addr = 0x0000;    
    i2c_address_slave(((u16Addr)>>8));
    while(1)
    {
        if(!i2c_is_slave_nak())
            break;
    }


    i2c_address_slave((u16Addr));
    while(1)
    {
        if(!i2c_is_slave_nak())
            break;
    }
#if 1
    //loopcnt = 0;
/*
    while(1)
    {
        i2c_gen_start_condition_S1();
        i2c_gen_24LC64_control();
        i2c_gen_rw_flag(I2C_WRITE);
        if(!i2c_is_slave_nak()||(loopcnt>5))
        {
            break;
        }
        loopcnt++;
       
    }
    */
    u8write  = 0xAC;
    i2c_write_byte(u8write);
    //i2c_master_gen_handshake(I2C_ACK);
    //i2c_gen_stop_condition_P();
    
    if(!i2c_is_slave_nak())
    {
        i2c_gen_stop_condition_P();
    }
    else
    {
        // nothing in the datasheet
        //i dont know;
    }
    
    //RT_BT_LOG(WHITE, I2C_WRITE_DATA, 2,u16Addr,u8write);
#endif
}

void i2c_test_24LC64()
{

#if 1    
    i2c_init();
  
    if(!check_i2c_idle(5))
    {
        return;
    }
    i2c_test_24LC64_byte_write();

    /*
    if(!check_i2c_idle(5))
    {
        return;
    }
    */
    


    
    //pf_delay(5);

    
    i2c_test_24LC64_random_read();
    if(!check_i2c_idle(5))
    {
        return;
    }
    
    /*
    i2c_test_24LC64_current_address_read();
    if(!check_i2c_idle(5))
    {
        return;
    } 
    */
#endif
       
}
#endif



 
#endif




