#ifndef _I2C_H_
#define _I2C_H_

#include "platform.h"
#include "DataType.h"
#include "mint_os.h"
#include "new_io.h"
#include "bb_driver.h"




//2 Registers for I2c control are located bgton 0x51 and 0x5c

/* i2c macro */
#define BTI2C_SCULL_P_NR_DEVS   1
enum I2C_OPERATION{I2C_WRITE=0, I2C_READ };// standard i2c spec
//enum I2C_OPERATION{I2C_READ=0, I2C_WRITE }; // for eeprom test only

enum I2C_HANDSHANK{I2C_ACK=0, I2C_NAK};
enum I2C_MODE{I2C_STARTARD=0, I2C_FAST, I2C_HS};//100kbps(10k pull-up), 400kbps(2.5k pull-up), 3.4Mbps(no support?)
enum I2C_CONDITION{I2C_START=0, I2C_STOP};// no use??
enum I2C_BUS_STATE{I2C_BUSY=0, I2C_IDLE};
enum I2C_PAD_CTRL_SEL{BT_CTRL=0, WIFI_CTRL};

//I2C return status
enum I2C_RETURN_STATUS{ I2C_SUCCESS             =0, 
                            I2C_DEVADDR_W_FAIL      =1, 
                            I2C_DEVADDR_R_FAIL      =2,
                            I2C_REGADDR_FAIL        =3,
                            I2C_READ_DATA_FAIL      =4,
                            I2C_WRITE_DATA_FAIL     =5,
                            I2C_ALLOC_MEMORY_FAIL   =6,
                            I2C_MODE_ERR            =7};



#ifdef _SUPPORT_8822B_I2C_

/* bt register macro*/
//#define BTON_REG_0xBC;
#define BTON_I2C_CONTROL_REGISTER   BTON_DLPS_CONTROL_REGISTER

#define PAGE0_0x10A8_REG 0x1A8 // pag0 0x10A8, 1 = page, 0 dont care, A8 offset
#define I2C_IO_READ() indirect_read_combo_dword_syson_reg(PAGE0_0x10A8_REG)// get gpio data
#define I2C_CHECK_NFC_PAD_CONTROLABLE ((indirect_read_combo_dword_syson_reg(PAGE0_0x10A8_REG)&BIT19)>>19)



typedef union PAGE0_NFC_PAD_CONTROL_S_{
    UINT32 d32;
    struct{
        UINT32 nfc_pad_data_in          :1; //[0]
        UINT32 nfc_pad_clk_in           :1; //[1]
        UINT32 nfc_pad_rfdisable_in     :1; //[2]
        UINT32 nfc_pad_int_in           :1; //[3]
        UINT32 WIFI_ONLY_18_4           :15;//[18:4]
        UINT32 nfc_pad_control_sel      :1; //[19]
        UINT32 WIFI_ONLY_31_20          :12;//[31:20]
    }b;

}PAGE0_NFC_PAD_CONTROL_S;

#endif

//===========================

#ifdef _SUPPORT_8723D_VENDOR_I2C_

/* bt register macro*/

//#define BTON_I2C_CONTROL_REGISTER   BTON_DLPS_CONTROL_REGISTER

//#define PAGE0_0x10A8_REG 0x1A8 // pag0 0x10A8, 1 = page, 0 dont care, A8 offset
//#define I2C_IO_READ() indirect_read_combo_dword_syson_reg(PAGE0_0x10A8_REG)// get gpio data
//#define I2C_CHECK_NFC_PAD_CONTROLABLE ((indirect_read_combo_dword_syson_reg(PAGE0_0x10A8_REG)&BIT19)>>19)

//#define I2C_IO_READ() indirect_read_combo_dword_syson_reg(PAGE0_0x10A8_REG)// get gpio data
// BT_GPIO16 = clock
// BT_GPIO17 = data
// BT_GPIO18 = interrupt



#endif

//===========================


typedef struct I2C_DELAY_TIME_PARA_S_ { 
   
    UINT16 I2C_AccessTime;        //I/O access time, nano second    
    UINT16 I2C_InitTime;          //I2C Init time, nano second    
    UINT16 I2C_StartHoldTime;     //Start bit hold time, nano second             
    UINT16 I2C_StartSetupTime;    //Start bit setup time, nano second    
    UINT16 I2C_StopHoldTime;      //Stop bit hold time, nano second    
    UINT16 I2C_StopSetupTime;     //Stop bit setup time, nano second    
    UINT16 I2C_SCLLowHoldTime;    //SCL LOW hold time, nano second    
    UINT16 I2C_SCLHIGHHoldTime;   //SCL high hold time, nano second    
    UINT16 I2C_BusIdleTime;       //BUS idle time between STOP and START condition, nano second        
}I2C_DELAY_TIME_PARA_S;




// function
void i2c_init();
void i2c_interrupt_handler();
void i2c_data_read(UINT32 u32length);
//void i2c_write_read();
void i2c_mode_sel(UINT8 mode);
void i2c_set_address(UINT8 addr);
void i2c_test_24LC64();
BOOLEAN i2c_is_bus_idle();
void i2c_test_24LC64();


#endif//_I2C_H_

