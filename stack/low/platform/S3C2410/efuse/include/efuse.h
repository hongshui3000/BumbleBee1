#ifndef _EFUSE_H_
#define _EFUSE_H_

#include "platform.h"
#include "DataType.h"


#define BT_EFUSE_REG_BASE_ADDR      0xb000a000

/*
 * Bits      |    ++Descriptin++
 * 31       |
 * 30:28  |
 * 27:24  |      
 * 23:20  |      
 * 19       |      
 * 18       |
 * 17:8    |      Efuse address.
 * 7:0      |	Efuse data.
**/
#define EFUSE_CTL1_REG               0x0144

/*
 * Bits      |    ++Descriptin++
 * 31:30  |    Efuse bank selection.
 * 29:0    |	Reserved.
**/
#define EFUSE_CTL2_REG               0x0010

#define EFUSE_BANK_1                 0x40
#define EFUSE_BANK_2                 0x80
#define EFUSE_BANK_3                 0xC0
#define EFUSE_TOTAL_LEN              958
#define EFUSE_BANK1_LEN              344
#define EFUSE_BANK2_LEN              256
#define EFUSE_BANK3_LEN              386

#define EFUSE_BYTE_READ(y)         	RD_8BIT_IO(BT_EFUSE_REG_BASE_ADDR, y)
#define EFUSE_BYTE_WRITE(y, z)     	WR_8BIT_IO(BT_EFUSE_REG_BASE_ADDR, y, z)
#define EFUSE_DWORD_READ(y)         RD_32BIT_IO(BT_EFUSE_REG_BASE_ADDR, y)
#define EFUSE_DWORD_WRITE(y, z)     WR_32BIT_IO(BT_EFUSE_REG_BASE_ADDR, y, z)

typedef struct _EFUSE_IO_CTRL_REG_S_ {
    UINT32 data:8;              /* bit[7:0], access data */
    UINT32 addr:10;             /* bit[17:8], access address */
    UINT32 autoload_en:1;       /* bit[18], autoload enable */
    UINT32 pwd_down:1;          /* bit[19], efuse power down */
    UINT32 setup_time:4;        /* bit[23:20], programming setup time, 
                                   125ns in units */
    UINT32 read_time:4;         /* bit[27:24], efuse read time. in the unit of 
                                   cycle time */
    UINT32 program_time:3;      /* bit[30:28], programming time */
    UINT32 access_flag:1;       /* bit[31], write "1" for program. 
                                            write "0" for read */
} EFUSE_IO_CTRL_REG_S;

UINT8 efuse_one_byte_read(UINT16 addr, UCHAR* data);
void start_update_parameter_from_efuse(void);
void read_efuse_bank_data(UINT16 size);
void efuse_init(void);

#ifdef _BT_ONLY_
#define EFUSE_PG_STATUS_OK              0
#define EFUSE_PG_STATUS_TIMEOUT         1
#define EFUSE_PG_STATUS_WRONG_PARAM     2

typedef struct EFUSE_MANAGE_S_ {
    UINT8 init;
    UINT8 cur_bank;  
    UINT8 cur_status;
    UINT8 auto_load_fail;
    UINT8 app_bank;
    UINT16 app_start_addr;
    UINT16 app_len;    
} EFUSE_MANAGE_S;

extern EFUSE_MANAGE_S efuse_manger;

#ifdef EFUSE_TEST_PG
void efuse_pg_test(void);
#endif

void efuse_page0_init(void);
UINT8 efuse_page0_set_bank(UINT8 bank);
void efuse_page0_write(UINT8 bank, UINT16 addr, UINT8 wdata);
UINT8 efuse_page0_read(UINT8 bank, UINT16 addr);
#endif

#endif//_OTP_H_

