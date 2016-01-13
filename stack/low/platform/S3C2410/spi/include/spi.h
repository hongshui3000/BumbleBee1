#ifndef _SPI_H_
#define _SPI_H_

#include "DataType.h"

#define SPI_REG_BASE                        0x40080000
#define SPI_REG_CTRLR0                      0x000
#define SPI_REG_CTRLR1                      0x004
#define SPI_REG_SSIENR                      0x008
#define SPI_REG_SER                         0x010
#define SPI_REG_BAUDR                       0x014
#define SPI_REG_TXFTLR                      0x018
#define SPI_REG_RXFTLR                      0x01C
#define SPI_REG_TXFLR                       0x020
#define SPI_REG_RXFLR                       0x024
#define SPI_REG_SR                          0x028
#define SPI_REG_IMR                         0x02C
#define SPI_REG_ISR                         0x030
#define SPI_REG_RISR                        0x034
#define SPI_REG_TXOICR                      0x038
#define SPI_REG_RXOICR                      0x03C
#define SPI_REG_RXUICR                      0x040
#define SPI_REG_MSTICR                      0x044
#define SPI_REG_ICR                         0x048
#define SPI_REG_IDR                         0x058
#define SPI_REG_SPIC_VERSION                0x05C
#define SPI_REG_DR_BASE                     0x060
#define SPI_REG_DR(n)                       (0x060 + ((n) << 2))
#define SPI_REG_READ_FAST_SINGLE            0x0E0
#define SPI_REG_READ_DUAL_DATA              0x0E4
#define SPI_REG_READ_DUAL_ADDR_DATA         0x0E8
#define SPI_REG_READ_QUAD_SINGLE            0x0EC
#define SPI_REG_READ_QUAD_ADDR_DATA         0x0F0
#define SPI_REG_WRITE_SIGNLE                0x0F4
#define SPI_REG_WRITE_DUAL_DATA             0x0F8
#define SPI_REG_WRITE_DUAL_ADDR_DATA        0x0FC
#define SPI_REG_WRITE_QUAD_DATA             0x100
#define SPI_REG_WRITE_QUAD_ADDR_DATA        0x104
#define SPI_REG_WRITE_ENABLE                0x108
#define SPI_REG_READ_STATUS                 0x10C
#define SPI_REG_CTRLR2                      0x110
#define SPI_REG_FBAUDR                      0x114
#define SPI_REG_ADDR_LENGTH                 0x118
#define SPI_REG_AUTO_LENGTH                 0x11C
#define SPI_REG_VALID_CMD                   0x120
#define SPI_REG_FLASH_SIZE                  0x124
#define SPI_REG_FLUSH_FIFO                  0x128

#ifdef _SPIC_FUNC_ENABLE_

#define RD_U32_SPI_REG(offset)        RD_32BIT_IO(SPI_REG_BASE, offset)
#define RD_U16_SPI_REG(offset)        RD_16BIT_IO(SPI_REG_BASE, offset)
#define RD_U8_SPI_REG(offset)         RD_8BIT_IO(SPI_REG_BASE, offset)
#define WR_U32_SPI_REG(offset, val)   WR_32BIT_IO(SPI_REG_BASE, offset, val)
#define WR_U16_SPI_REG(offset, val)   WR_16BIT_IO(SPI_REG_BASE, offset, val)
#define WR_U8_SPI_REG(offset, val)    WR_8BIT_IO(SPI_REG_BASE, offset, val)

#define SPI_RDSR_BSY            0x01
#define SPI_RDSR_WEL            0x02


enum SPI_FLASH_ERASE_TYPE {
    SPI_FLASH_ERASE_TYPE_CHIP   = 0,
    SPI_FLASH_ERASE_TYPE_BLOCK  = 1,        
    SPI_FLASH_ERASE_TYPE_SECTOR = 2,
};

enum SPI_FLASH_MXIC_OPCODES {
    SPI_FLASH_MXIC_OPCODE_WRSR = 0x01,    
    SPI_FLASH_MXIC_OPCODE_PP   = 0x02,     
    SPI_FLASH_MXIC_OPCODE_READ = 0x03, 
    SPI_FLASH_MXIC_OPCODE_RDSR = 0x05, 
    SPI_FLASH_MXIC_OPCODE_WREN = 0x06,    
    SPI_FLASH_MXIC_OPCODE_REMS = 0x90,     
    SPI_FLASH_MXIC_OPCODE_RDID = 0x9F,     
    SPI_FLASH_MXIC_OPCODE_SE   = 0x20, 
    SPI_FLASH_MXIC_OPCODE_BE   = 0xD8, 
    SPI_FLASH_MXIC_OPCODE_CE   = 0xC7, 
};

#define SPI_WAIT_BUSY           while (RD_U32_SPI_REG(SPI_REG_SR) & 0x01)

/* the structure of control 0 register in user mode */
typedef struct SPI_REG_CTRLR0_S_ {
    UINT32 rsvd0:6;     /* bit[5:0], reserved */
    UINT32 scph:1;      /* bit[6], spi clock toggles at start/middle of first data bit */   
    UINT32 scpol:1;     /* bit[7], spi clock is high or low in inactive state */
    UINT32 tmod:2;      /* bit[9:8], 00B is tx mode and otherwise is rx mode*/
    UINT32 rsvd1:6;     /* bit[15:10], reserved */
    UINT32 addr_ch:2;   /* bit[17:16], 0/1: single, 2:dual, 3:quad channels */
    UINT32 data_ch:2;   /* bit[19:18], 0/1: single, 2:dual, 3:quad channels */
    UINT32 fast_rd:1;   /* bit[20], in fast read mode or not  */
    UINT32 rsvd2:11;    /* bit[31:21], reserved */
} SPI_REG_CTRLR0_S, *PSPI_REG_CTRLR0_S;

extern UINT8 is_spi_init;

void spi_init(void);
UINT32 spi_get_rdid(void);
UINT16 spi_get_rems();
void spi_read_data(UINT32 addr, UINT16 read_len, UINT8 * rdata);
void spi_write_data(UINT32 addr, UINT16 write_len, UINT8 * wdata);
void spi_write_enable(void);
void spi_write_status_register(UINT8 reg);
UINT8 spi_read_status_register(void);
void spi_erase(UINT8 type, UINT32 start_addr, UINT32 end_addr);
void spi_test_func(void);

#define SWAP2B(_val)	(((((UINT16)(_val))&0x00ff)<<8) | \
 						  ((((UINT16)(_val))&0xff00)>>8))
#define SWAP4B(_val)	(((((UINT32)(_val))&0x000000ff)<<24)	|	\
						  ((((UINT32)(_val))&0x0000ff00)<<8)	|	\
						  ((((UINT32)(_val))&0x00ff0000)>>8)	|	\
						  ((((UINT32)(_val))&0xff000000)>>24))
#endif

#endif

