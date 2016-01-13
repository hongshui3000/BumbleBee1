/********************************* Logger *************************/ 
enum { __FILE_NUM__= 160 };
/********************************* Logger *************************/

/* ========================= Include File Section ========================= */

#include "spi.h"
#include "logger.h"
#include "platform.h"

#ifdef _SPIC_FUNC_ENABLE_
UINT8 is_spi_init = FALSE;

/**************************************************************************
 * Function     : spi_init
 *
 * Description  : This function is used to initiate spi contorller
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_init(void)
{
    UINT32 reg_value;

#ifdef _IS_ASIC_
    /* enable SPI LDO and disable isolation */    
    reg_value = VENDOR_READ(0x70);
#ifdef _RTL8821B_TC_SPECIFIC_ 
    reg_value |= BIT1;
#else
    reg_value |= BIT0;
#endif
    reg_value &= ~BIT16;
    VENDOR_WRITE(0x70, reg_value);
      
    /* Wait for LDO stable, ~10us, 200us is recommended */
    pf_delay(1);
        
    reg_value = VENDOR_READ(0x74);
    reg_value &= ~BIT24;
    VENDOR_WRITE(0x74, reg_value);    
#endif

    /* enable SPI PLL */
    reg_value = VENDOR_READ(0x70);    
    if (!(reg_value & BIT16))
    {
        VENDOR_WRITE(0x70, reg_value | BIT16);
    }
    
    /* disable SPIC */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 0);

    /* set spi clock - 1/2 sysclk */
    WR_U32_SPI_REG(SPI_REG_BAUDR, 1);    

    /* single channel, blocking write */
    WR_U32_SPI_REG(SPI_REG_VALID_CMD, 0x200);

    /* set SER */
    WR_U32_SPI_REG(SPI_REG_SER, 1);  

    /* mask all spi intrerupts (do not enable) */
    WR_U32_SPI_REG(SPI_REG_IMR, 0); 

    /* set Dummy cycle */
#ifdef _IS_ASIC_
    WR_U32_SPI_REG(SPI_REG_AUTO_LENGTH, 2); 
#else
    WR_U32_SPI_REG(SPI_REG_AUTO_LENGTH, 0x30000); 
#endif

    /* enable SPIC */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 1);

    /* do global unprotect for serial flash */
    spi_write_status_register(0x00);

    is_spi_init = TRUE;
}

/**************************************************************************
 * Function     : spi_flash_tx_cmd
 *
 * Description  : This function is used to initiate a tx command. The command 
 *                means opcode + address or opcode + status in command/address
 *                phase in SPIC. The data means any data sent to spi flash in 
 *                data phase. (Note: data_buf must be 4 byte alignmnet 
 *                when data_len is larger than 3)              
 *
 * Parameters   : cmd_len: the length of data in command/address phase (1 ~ 4)
 *                data_len: the length of data in data phase (0 ~ N)
 *                *cmd_buf: the pointer of command buffer
 *                *data_buf: the pointer of data buffer
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_flash_tx_cmd(UINT8 cmd_len, UINT16 data_len, UINT8* cmd_buf, UINT8* data_buf)
{
    UINT32 temp;
    UINT8 i = 0;
    UINT16 offset = 0;
    SPI_REG_CTRLR0_S reg_ctrl0;

    /* disable SPIC */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 0);

    /* disable SER */
    WR_U32_SPI_REG(SPI_REG_SER, 0);

    /* set BAUDR, FBAUDR, TXFTLR, RXFTLR ... */
    
    /* set CTRLR0~2, SSIENR, ADDR_LENGTH, AUTO_LENGTH, DR ... */

    /* set transfer mode (tx mode), no fast read, support mode 3 */
    *(UINT32*)&reg_ctrl0 = 0;
    reg_ctrl0.scpol = 1;
    reg_ctrl0.scph = 1;    
    WR_U32_SPI_REG(SPI_REG_CTRLR0, *(UINT32*)&reg_ctrl0);

    if (cmd_len > 1)
    {
        /* write byte numbers of address phase */
        WR_U32_SPI_REG(SPI_REG_ADDR_LENGTH, cmd_len - 1);  
    }  

    /* write spi data port by one byte unit (big endian in register) */
    while (cmd_len)
    {
        WR_U8_SPI_REG(SPI_REG_DR(i), cmd_buf[offset]);
        offset++;            
        cmd_len--;
    }          

    /* enable SER */
    WR_U32_SPI_REG(SPI_REG_SER, 1);

    i = 1;
    offset = 0;

    /* set byte length of ADDR phase - after transmit one byte cmd */

    /* write DR with transmitting command or data */

    /* write spi data port by four byte unit (big endian in register) */   
    while (data_len > 3)
    {
        temp = *(UINT32*)&data_buf[offset];
        WR_U32_SPI_REG(SPI_REG_DR(i), temp);
        offset += 4;
        data_len -= 4;
        i = (i + 1) & 0x1F;        
    }

    /* write spi data port by one byte unit (big endian in register) */
    while (data_len)
    {
        WR_U8_SPI_REG(SPI_REG_DR(i), data_buf[offset]);
        offset++;            
        data_len--;
    } 

    /* write SSIENR */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 1);

    /* check SR_BUSY */
    do
    {
        temp = RD_U32_SPI_REG(SPI_REG_SR);
    }
    while (temp & BIT0);
}

/**************************************************************************
 * Function     : spi_flash_rx_cmd
 *
 * Description  : This function is used to initiate a rx command. In some flash
 *                command, we can receive read data from spi flash after we 
 *                send opcode and something. In this case, we can use rx 
 *                command to implement it.(Note: tx_buf must be 4 byte alignmnet 
 *                when tx_len is larger than 3; rx_buf must be 4 byte alignmnet 
 *                when tx_len is larger than 3)
 *
 * Parameters   : tx_len: the length of sent data (valid 1 ~ 4)
 *                rx_len: the length of read data (valid 1 ~ N)
 *                *tx_buf: the pointer of sent data buffer
 *                *rx_buf: the pointer of received data buffer
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_flash_rx_cmd(UINT8 tx_len, UINT16 rx_len, UINT8* tx_buf, UINT8 * rx_buf)
{
    UINT32 temp;
    UINT8 i = 0;
    UINT16 offset = 0;
    UINT8 addr_len;
    SPI_REG_CTRLR0_S reg_ctrl0;    

    /* disable SPIC */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 0);

    /* disable SER */
    WR_U32_SPI_REG(SPI_REG_SER, 0);

    /* set BAUDR, FBAUDR, TXFTLR, RXFTLR ... */
    
    /* set CTRLR0~2, SSIENR, ADDR_LENGTH, AUTO_LENGTH, DR ... */
    
    /* set transfer mode (rx mode), no fast read, support mode 3 */
    *(UINT32*)&reg_ctrl0 = 0;
    reg_ctrl0.tmod = 3;
    reg_ctrl0.scpol = 1;
    reg_ctrl0.scph = 1;      
    WR_U32_SPI_REG(SPI_REG_CTRLR0, *(UINT32*)&reg_ctrl0);       

    if (tx_len > 1)
    {
        /* write byte numbers of address phase */
        addr_len = MIN(3, tx_len - 1);
        WR_U32_SPI_REG(SPI_REG_ADDR_LENGTH, addr_len);  
    }    

    /* set frame number of receiving data */       
    WR_U32_SPI_REG(SPI_REG_CTRLR1, rx_len);

    /* enable SER */
    WR_U32_SPI_REG(SPI_REG_SER, 1);

    /* write DR with transmitting command or data */

    /* set byte length of ADDR phase - after transmit one byte cmd */

    /* send command or tx data */
    while (tx_len > 3)
    {
        temp = *(UINT32*)&tx_buf[offset];
        WR_U32_SPI_REG(SPI_REG_DR(i), temp);
        offset += 4;
        tx_len -= 4;
        i = (i + 1) & 0x1F;        
    }

    while (tx_len)
    {
        WR_U8_SPI_REG(SPI_REG_DR(i), tx_buf[offset]);
        offset++;            
        tx_len--;
    } 

    /* write SSIENR */
    WR_U32_SPI_REG(SPI_REG_SSIENR, 1);

    /* check SR_BUSY */
    do
    {
        temp = RD_U32_SPI_REG(SPI_REG_SR);
    }
    while (temp & BIT0);

    /* read DR after send receive commnad */
    offset = 0;
    i = 0;

    /* read spi data port by four byte unit (big endian in register) */
    while (rx_len > 3)
    {
        *(UINT32*)&rx_buf[offset] = RD_U32_SPI_REG(SPI_REG_DR(i));
        offset += 4;
        rx_len -= 4;
        i = (i + 1) & 0x1F;
    }

    /* read spi data port by one byte unit (big endian in register) */
    while (rx_len)
    {
        rx_buf[offset] = RD_U8_SPI_REG(SPI_REG_DR(i) + rx_len - 1);
        offset++;            
        rx_len--;
    }     
}

/**************************************************************************
 * Function     : spi_get_rdid
 *
 * Description  : This function is used to send RDID command to SPI flash.
 *                The flash can sent back 24 bit RDID content after we call this
 *                function
 *
 * Parameters   : None
 *
 * Returns      : The content of RDID (bit[23:0] is valid)
 *
 *************************************************************************/
UINT32 spi_get_rdid(void)
{
    UINT32 rbuf;
    UINT8 wbuf = SPI_FLASH_MXIC_OPCODE_RDID;    
    spi_flash_rx_cmd(1, 3, &wbuf, (UINT8*)&rbuf); 
    return rbuf;
}

/**************************************************************************
 * Function     : spi_get_rems
 *
 * Description  : This function is used to send REMS command to SPI flash.
 *                The flash can sent back 16 bit REMS content after we call this
 *                function
 *
 * Parameters   : None
 *
 * Returns      : The content of REMS (bit[15:0] is valid)
 *
 *************************************************************************/
UINT16 spi_get_rems(void)
{
    UINT16 rbuf;
    UINT32 wbuf = (0x01 << 24) | SPI_FLASH_MXIC_OPCODE_REMS;    
    spi_flash_rx_cmd(4, 2, (UINT8*)&wbuf, (UINT8*)&rbuf); 
    return rbuf;    
}

/**************************************************************************
 * Function     : spi_read_data
 *
 * Description  : This function is used to send READ Data command to SPI 
 *                flash. The flash can sent back burst read data from assigned
 *                spi flash physical address after we call this function
 *               (note: the pointer of rdata must 4 byte alignment when 
 *                 read_len is larger than 3)
 *
 * Parameters   : addr: the 24 bite physical address of spi flash
 *                read_len: read length
 *                rdata: the pointer of read buffer
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_read_data(UINT32 addr, UINT16 read_len, UINT8 *rdata)
{    
    UINT32 wbuf = (SPI_FLASH_MXIC_OPCODE_READ << 24) | addr;
    wbuf = SWAP4B(wbuf);    
    spi_flash_rx_cmd(4, read_len, (UINT8*)&wbuf, rdata); 
}

/**************************************************************************
 * Function     : spi_write_data
 *
 * Description  : This function is used to send Page Program command to
 *                SPI flash. We can write data into spi flash from assigned
 *                spi flash physical address after we call this function
 *                (note: the pointer of wdata must 4 byte alignment when 
 *                 write_len is larger than 3)
 *
 * Parameters   : addr: the 24 bite physical address of spi flash
 *                write_len: write length
 *                wdata: the pointer of write buffer
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_write_data(UINT32 addr, UINT16 write_len, UINT8 *wdata)
{
    UINT8 cmd_buf[4];
    
    cmd_buf[0] = SPI_FLASH_MXIC_OPCODE_PP;
    cmd_buf[1] = addr >> 16;
    cmd_buf[2] = addr >> 8;
    cmd_buf[3] = addr;    

    spi_write_enable();
    spi_flash_tx_cmd(4, write_len, cmd_buf, wdata);
    while (spi_read_status_register() & (SPI_RDSR_BSY | SPI_RDSR_WEL));
}

/**************************************************************************
 * Function     : spi_read_status_register
 *
 * Description  : This function is used to send Read Status Register Commnad 
 *                to spi flash. Then spi flash will send back 8 bit status 
 *                for us (the bit definition is depend on flash vendor)
 *
 * Parameters   : None
 *
 * Returns      : 8 bite content of READ_STATUS_REGISTER in spi flash
 *
 *************************************************************************/
UINT8 spi_read_status_register(void)
{    
    UINT8 wbuf = SPI_FLASH_MXIC_OPCODE_RDSR;
    UINT8 rbuf;
    spi_flash_rx_cmd(1, 1, &wbuf, &rbuf);
    return rbuf;
}

/**************************************************************************
 * Function     : spi_write_enable
 *
 * Description  : This function is used to send Write Enable Commnad 
 *                to spi flash. Then spi flash will set its WREN bit.
 *                This function must be called before we want to do page 
 *                program,erase, or any write operation
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_write_enable(void)
{
    UINT8 wbuf = SPI_FLASH_MXIC_OPCODE_WREN;   
    
    spi_flash_tx_cmd(1, 0, &wbuf, NULL);
    while ((spi_read_status_register() & 
                (SPI_RDSR_BSY | SPI_RDSR_WEL)) != SPI_RDSR_WEL);
}

/**************************************************************************
 * Function     : spi_write_status_register
 *
 * Description  : This function is used to send Write Status Commnad and 
 *                modified content to spi flash. Then spi flash will configure 
 *                something inside. Usually, we need to read status register to 
 *                check and confirm corresponding bit(s).
 *
 * Parameters   : reg: 8 bit register after Write Status Register OpCode
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_write_status_register(UINT8 reg)
{    
    UINT8 wbuf[2]; 
    
    wbuf[0] = SPI_FLASH_MXIC_OPCODE_WRSR; 
    wbuf[1] = reg;
    spi_write_enable();    
    spi_flash_tx_cmd(2, 0, wbuf, NULL);
    while (spi_read_status_register() & (SPI_RDSR_BSY | SPI_RDSR_WEL));
}

/**************************************************************************
 * Function     : spi_erase
 *
 * Description  : This function is used to send varied Erase Command  to spi 
 *                flash. When erase type is not CHIP Erase, we can set start
 *                and end address to erase assigned area in the spi flash.
 *
 * Parameters   : type: Erase Type (Sector Erase / Block Erase / Chip Erase) 
 *                start_addr: the 24 bit spi physical address in the first 
 *                            erase block
 *                end_addr: the 24 bit spi physical address in the last 
 *                            erase block
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_erase(UINT8 type, UINT32 start_addr, UINT32 end_addr)
{    
    UINT8 opcode;
    UINT8 shift_right;
    UINT32 wbuf;
    UINT32 block_index;
    UINT32 start_block;
    UINT32 end_block;

    if (type == SPI_FLASH_ERASE_TYPE_SECTOR)
    {
        /* erase 4KB */
        opcode = SPI_FLASH_MXIC_OPCODE_SE;
        shift_right = 12;
        start_block = start_addr >> 12;
        end_block = end_addr >> 12;
    }
    else if (type == SPI_FLASH_ERASE_TYPE_BLOCK)
    {
        /* erase 64KB */
        opcode = SPI_FLASH_MXIC_OPCODE_BE;
        shift_right = 16;
        start_block = start_addr >> 16;
        end_block = end_addr >> 16;
    }
    else
    {
        /* chip erase */
        spi_write_enable();
        opcode = SPI_FLASH_MXIC_OPCODE_CE;
        spi_flash_tx_cmd(1, 0, &opcode, NULL);
        while (spi_read_status_register() & (SPI_RDSR_BSY | SPI_RDSR_WEL));
        return;
    }

    for (block_index = start_block; block_index < end_block; block_index++)
    {
        spi_write_enable();
        wbuf = (opcode << 24) | (block_index << shift_right);
        wbuf = SWAP4B(wbuf);
        spi_flash_tx_cmd(4, 0, (UINT8*)&wbuf, NULL);
        while (spi_read_status_register() & (SPI_RDSR_BSY | SPI_RDSR_WEL));
    }
}

#ifdef _SPIC_TEST_
/**************************************************************************
 * Function     : spi_test_func
 *
 * Description  : This function is used to generate a test code for testing.
 *
 * Parameters   : None
 *
 * Returns      : None
 *
 *************************************************************************/
void spi_test_func(void)
{
    UINT32 temp, temp2;
    UINT32 temp_buf[3];
    
    spi_init();

    /* read rdid */
    temp = spi_get_rdid();
    temp2 = spi_get_rems();    
    RT_TMP_LOG("spi rdid = %08x rems = %08x\n", temp, temp2); 

    temp = 0;
    spi_read_data(0, 4, (UINT8*)&temp);    
    RT_TMP_LOG("1.spi read addr 0x000000 data = %08x\n", temp);  

    temp = 0;
    spi_read_data(4, 4, (UINT8*)&temp);    
    RT_TMP_LOG("1.spi read addr 0x000004 data = %08x\n", temp);

    temp = 0;
    spi_read_data(8, 4, (UINT8*)&temp);    
    RT_TMP_LOG("1.spi read addr 0x000008 data = %08x\n", temp);

    temp = 0;
    spi_read_data(0, 1, (UINT8*)&temp);
    RT_TMP_LOG("2-1.spi read addr 0x000000 data = %02x\n", temp); 

    temp = 0;
    spi_read_data(1, 1, (UINT8*)&temp);
    RT_TMP_LOG("2-2.spi read addr 0x000001 data = %02x\n", temp); 
    temp = 0;
    spi_read_data(2, 1, (UINT8*)&temp);
    RT_TMP_LOG("2-3.spi read addr 0x000002 data = %02x\n", temp); 

    temp = 0;
    spi_read_data(3, 1, (UINT8*)&temp);
    RT_TMP_LOG("2-4.spi read addr 0x000003 data = %02x\n", temp); 

    spi_read_data(0, 12, (UINT8*)temp_buf);    
    RT_TMP_LOG("1.spi read addr 0x000000 data = %08x %08x %08x\n", temp_buf[0], temp_buf[1], temp_buf[2]);  

    
    temp_buf[0] = 0x12345678;
    temp_buf[1] = 0xabcdef01;
    temp_buf[2] = 0x55aaccdd;
    spi_write_data(0x020202, 12, (UINT8*)temp_buf);

    spi_read_data(0x020202, 12, (UINT8*)temp_buf);    
    RT_TMP_LOG("2.spi read addr 0x000000 data = %08x %08x %08x\n", temp_buf[0], temp_buf[1], temp_buf[2]);  


    //spi_erase(SPI_FLASH_ERASE_TYPE_CHIP, 0, 0);
    spi_erase(SPI_FLASH_ERASE_TYPE_BLOCK, 0, 0x2FFFF);

    spi_read_data(0x020202, 12, (UINT8*)temp_buf);    
    RT_TMP_LOG("3.spi read addr 0x000000 data = %08x %08x %08x\n", temp_buf[0], temp_buf[1], temp_buf[2]);  

    temp_buf[0] = 0x12345678;
    temp_buf[1] = 0xabcdef01;
    temp_buf[2] = 0x55aaccdd;
    spi_write_data(0x020202, 12, (UINT8*)temp_buf);

    spi_read_data(0x020202, 12, (UINT8*)temp_buf);      
    RT_TMP_LOG("4.spi read addr 0x000000 data = %08x %08x %08x\n", temp_buf[0], temp_buf[1], temp_buf[2]);  
}
#endif
#endif


