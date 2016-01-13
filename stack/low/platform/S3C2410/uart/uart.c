/***************************************************************************
 Copyright (C) MindTree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/
enum { __FILE_NUM__= 106 };


//#if defined(ENABLE_LOGGER) || defined(DIRECT_UART_PRINT)

#include "platform.h"
#include "uart.h"
#include "S3C2410_task.h"
#include "logger.h"
#include "hci_vendor_defines.h"
#include "otp.h"
#include "dma_usb.h"
#include "lbm.h"

UART_T  Uart;
UINT8 is_uart_init;
UINT8   uart_rev_len;
UINT16 uart_dbg_wptr;
UINT8 uart_dbg_data[REV_FIFO_LEN];

void  S3C2410UartTrans_TxData(void)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_
	UCHAR           *buf_rd_ptr;
	UINT16          len_available_in_fifo;
	UINT16          length_to_write;
	UINT32          n;

	/*
	Interrupt enable Register
	7: THRE Interrupt Mode Enable
	2: Enable Receiver Line Status Interrupt
	1: Enable Transmit Holding Register Empty Interrupt
	0: Enable Received Data Available Interrupt
	*/
	// disable all interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0);    

	buf_rd_ptr = (UCHAR *)(Uart.DataPtr + Uart.LenWritten);

	// FIFO available length
	len_available_in_fifo = UART_FIFO_MAX_LENGTH - UART_READ(UART_TFL_OFF);
	length_to_write = (Uart.Len - Uart.LenWritten);

	if (length_to_write > len_available_in_fifo)
	{
		length_to_write = len_available_in_fifo;
	}
	Uart.LenWritten += length_to_write;

	// Load the data into the UART1 fifo
	for(n=0; n<length_to_write; n++)
	{
		// push data to UART FIFO
		UART_WRITE(UART_SEND_BUF_OFF, buf_rd_ptr[n]);     
	}

	// enable UART1 Interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x83);    

	return;
#endif    
}

BOOLEAN S3C2410UartTransWr(HCI_TRANSPORT_PKT_TYPE pkt_type, UCHAR *buf, UINT16 len)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_

	/* Check the state of the UART to see if it is free */
	if(Uart.State != UART_FREE)
	{
		/* Platform logger transport not initialized yet!! */
		BZ_ASSERT_NOL(0);
		return FALSE;
	}

	Uart.State = UART_TRANSMITTING;

	/* Store the Data to be written in the UART's buffer */
	Uart.DataPtr = buf;
	Uart.Len = len;
	Uart.LenWritten = 0;

	UART_WRITE(UART_SEND_BUF_OFF, pkt_type);

	S3C2410UartTrans_TxData();
#endif  
	return TRUE;  
}

void MINT_trans_uart_tx(void)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_

	// disable UART1 Interrupts

	if(Uart.Len != 0) 
	{
		if(Uart.LenWritten == Uart.Len) // One FULL Packet has been Transmitted
		{
			UINT16 len;

			len = Uart.Len;
			// RESET the Transport Logic Structures for a NEW Pkt.
			Uart.LenWritten = 0;
			Uart.Len = 0;
			Uart.State = UART_FREE;

			// Call the callback to indicate that the previous pkt
			// transmission has finished
			Uart.TxCompFunc(Uart.DataPtr, len, 0);
			UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x01);    
		}
		else
		{
			S3C2410UartTrans_TxData();
		}
	}
	else
	{
		Uart.LenWritten = 0;
		Uart.Len = 0;
		Uart.State = UART_FREE;
		UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x01);    
	}
#endif    
}

BOOLEAN uart_trans_transport_is_free(void)
{
	return (BOOLEAN)(Uart.State == UART_FREE);
}

#ifdef ENABLE_LOGGER
void S3C2410UartLog_TxData(void)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_
	UCHAR           *buf_rd_ptr;
	UINT16          len_available_in_fifo;
	UINT16          length_to_write;
	UINT32          n = 0;

	/*
	Interrupt enable Register
	7: THRE Interrupt Mode Enable
	2: Enable Receiver Line Status Interrupt
	1: Enable Transmit Holding Register Empty Interrupt
	0: Enable Received Data Available Interrupt
	*/
	// disable all interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0);    

	buf_rd_ptr = (UCHAR *)(Uart.DataPtr + Uart.LenWritten);

	// FIFO available length
	len_available_in_fifo = UART_FIFO_MAX_LENGTH-UART_READ(UART_TFL_OFF);
	length_to_write = (Uart.Len - Uart.LenWritten);

	if (length_to_write > len_available_in_fifo)
	{
		length_to_write = len_available_in_fifo;
	}
	Uart.LenWritten += length_to_write;

	// Load the data into the UART1 fifo
	for(n=0; n<length_to_write; n++)
	{
		// push data to UART FIFO
		UART_WRITE(UART_SEND_BUF_OFF, buf_rd_ptr[n]);     
	}

	// enable UART1 Interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x87);

	
	return;
#endif    
}

BOOLEAN uart_logger_transport_is_free(void)
{
    if (IS_SUPPORT_HCI_LOG_PKT &&
        (dbg_vendor_log_interface != VENDOR_LOG_PACKET_TYPE_INVALID))
    {
    	return (BOOLEAN)(dbg_vendor_log_buf == NULL);               
    }
    else
    {
    	return (BOOLEAN)(Uart.State == UART_FREE);        
    }
}

BOOLEAN S3C2410UartLogInit(UART_TX_COMP_FUNC_PTR tx_comp_func, 
                                    UART_DBG_RECD_FUNC_PTR dbg_rx_func)
{
	if (tx_comp_func == NULL)
	{
		Uart.State = UART_NOT_INIT;
	}
	else
	{
		Uart.State = UART_FREE;
	}

	/* Populate the Global UART structure with the call backs */
	Uart.TxCompFunc = tx_comp_func;
#ifdef _UART_RX_ENABLE_
    Uart.RxFunc = dbg_rx_func;
#endif
	Uart.DataPtr = NULL;
	Uart.Len = 0;
	Uart.LenWritten = 0;

	// UART init function
	UARTInit(otp_str_data.SYS_log_uart_baudrate, UART_PARITY_DISABLE, UART_STOP_1BIT,
		UART_DATA_LEN_8BIT, 0x81, 0x05);

	return TRUE;
}

BOOLEAN S3C2410UartLogWr(UCHAR *buf, UINT16 len)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_
	/* Check the state of the UART to see if it is free */
	if (Uart.State != UART_FREE)
	{
		/* Platform logger transport not initialized yet!! */
		BZ_ASSERT_NOL(0);
		return FALSE;
	}

	Uart.State = UART_TRANSMITTING;
	/* Store the Data to be written in the UART's buffer */
	Uart.DataPtr = buf;
	Uart.Len = len;
	Uart.LenWritten = 0;

	S3C2410UartLog_TxData();
#endif

	return TRUE;
}

void MINT_logger_uart_tx(void)
{
#ifndef _DONT_USE_LOG_UART_TX_INT_
	if (Uart.Len != 0) 
	{
		if (Uart.LenWritten == Uart.Len) // One FULL Packet has been Transmitted
		{
			UINT16 len;

			len = Uart.Len;
			// RESET the Transport Logic Structures for a NEW Pkt.
			Uart.LenWritten = 0;
			Uart.Len = 0;
			Uart.State = UART_FREE;

			// Call the callback to indicate that the previous pkt
			// transmission has finished
			Uart.TxCompFunc(Uart.DataPtr, len, 0);
		}
		else
		{
			S3C2410UartLog_TxData();
		}
	}
	else
	{
		Uart.LenWritten = 0;
		Uart.Len = 0;
		Uart.State = UART_FREE;
	}
#endif    
}
#endif // ENABLE_LOGGER


// UART interrupt handler
SECTION_ISR_LOW void UartIntrHandler(void)
{
	UINT32 interrupt_status;
	UINT32 fifo_enable;
	UINT32 line_status;
	UINT32 read_data;
	UINT8  data_len;
	UINT8  index;
    UINT16 wptr;


	// UART interrupt
	interrupt_status = UART_READ(UART_INTERRUPT_IDEN_REG_OFF);
	fifo_enable = interrupt_status & 0xc0;
	interrupt_status = interrupt_status & 0x0f;

	// disable UART1 Interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0); 

	switch(interrupt_status)
	{
	case 0x04:	// UART received data available interrupt or character timeout interrupt
	case 0x0c:
		{
            wptr = uart_dbg_wptr;
			if (fifo_enable != 0)
			{
				data_len = UART_READ(UART_RFL_OFF);
                
                for(index=0; index<data_len; index++)
				{
					uart_dbg_data[wptr] = UART_READ(UART_REV_BUF_OFF);
                    wptr = (wptr + 1) & (REV_FIFO_LEN - 1);
				}

#ifdef _UART_RX_ENABLE_
                Uart.RxFunc(&uart_dbg_data[uart_dbg_wptr], data_len);
#endif
			}
			else
			{
			    line_status = UART_READ(UART_LINE_STATUS_REG_OFF);
				if((line_status & 0x01) == 1)
				{
					uart_dbg_data[wptr] = UART_READ(UART_REV_BUF_OFF);
                    wptr = (wptr + 1) & (REV_FIFO_LEN - 1);

#ifdef ENABLE_LOGGER
#ifdef _UART_RX_ENABLE_
                    uart_rev_len++;
                    
					if (uart_rev_len == UART_FIFO_MAX_LENGTH)
					{
						Uart.RxFunc(&uart_dbg_data[uart_dbg_wptr], UART_FIFO_MAX_LENGTH);
						uart_rev_len = 0;
					}
#endif                    
#endif
				}
			}
            uart_dbg_wptr = wptr;
		}

		break;

	case 0x06:  //Receiver Line Status interrupt
		{
			line_status = UART_READ(UART_LINE_STATUS_REG_OFF);
		}
		break;

	case 0x02:  //Transmit Holding Register empty
#ifndef _DONT_USE_LOG_UART_TX_INT_
		if(Uart.Len != 0 && Uart.LenWritten == Uart.Len)
		{
#ifdef ENABLE_LOGGER
            MINT_logger_uart_tx();
#endif //ENABLE_LOGGER
		}
		else
		{
			S3C2410_task_send_signal(UART_TX_ISR_TO_PF_TASK_SIGNAL, NULL);
			UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x01); 
			return;
		}
#endif        
		break;

	case 0x0:  //Modem status register interrupt
		{
			read_data = UART_READ(UART_MODEM_STATUS_REG_OFF);
		}
		break;

	case 0x07:  //Busy detect indication interrupt
		{
			read_data = UART_READ(UART_STATUS_REG_OFF);
		}
		break;

	default:
		break;
	}

	// enable UART Interrupts
    UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0x05); 
	return;
}

UINT32 UARTInit(UINT32 BaudRate, UINT8 parity, UINT8 stop, UINT8 DataLength, 
				UINT32 FIFOControl, UINT32 IntEnReg)
{
	UINT32 set_data;
	UINT32 divisor;
	UINT32 dlh;
	UINT32 dll;
    GENERAL_CONTROL_S_TYPE bt_general_ctrl;
    //the setting value loaded from efuse
    bt_general_ctrl.d16 = otp_str_data.general_control;


	/*
	Interrupt enable Register
	7: THRE Interrupt Mode Enable
	2: Enable Receiver Line Status Interrupt
	1: Enable Transmit Holding Register Empty Interrupt
	0: Enable Received Data Available Interrupt
	*/
	// disable all interrupts
	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, 0);

	/*
	Line Control Register
	7:   DLAB, enable reading and writing DLL and DLH register, and must be cleared after
	initial baud rate setup
	3:   PEN, parity enable/disable
	2:   STOP, stop bit
	1:0  DLS, data length
	*/

	// set DLAB bit to 1
	UART_WRITE(UART_LINE_CTL_REG_OFF, 0x80);

	// set up buad rate division 
	divisor = (otp_str_data.crystal_clk / (BaudRate << 4));

	dll = divisor & 0xff;
	dlh = (divisor & 0xff00)>>8;
	UART_WRITE(UART_DLL_OFF, dll);
	UART_WRITE(UART_DLH_OFF, dlh);

	// clear DLAB bit 
	UART_WRITE(UART_LINE_CTL_REG_OFF, 0);

	// set data format
	set_data = parity | stop | DataLength;
	UART_WRITE(UART_LINE_CTL_REG_OFF, set_data);

	/* FIFO Control Register
	7:6  level of receive data available interrupt
	5:4  level of TX empty trigger
	2    XMIT FIFO reset
	1    RCVR FIFO reset
	0    FIFO enable/disable
	*/
	// FIFO setting, enable FIFO and set trigger level (2 less than full when receive
	// and empty when transfer 
	UART_WRITE(UART_FIFO_CTL_REG_OFF, FIFOControl);

	/*
	Interrupt Enable Register
	7: THRE Interrupt Mode enable
	2: Enable Receiver Line status Interrupt
	1: Enable Transmit Holding register empty INT32
	0: Enable received data available interrupt
	*/
	//Disable rx interrupt
    if (!bt_general_ctrl.b.log_uart_rx_en)
        IntEnReg &= (~0x5);

  	UART_WRITE(UART_INTERRUPT_EN_REG_OFF, IntEnReg);

    uart_rev_len = 0;
    uart_dbg_wptr = 0;
    is_uart_init = TRUE;

	return 0;
}

UINT32 UARTBytePut(UINT8 data)
{
    UINT32 read_value;
    while (1)
    {
        read_value = UART_READ(UART_LINE_STATUS_REG_OFF);
        if (read_value & 0x60)
        {
            /* we can break if bit5 or bit6 set to 1 - suggested by kevin
               (austin) */
            break;
        }
    }
	UART_WRITE(UART_SEND_BUF_OFF, data);

	return 0;
}

#ifdef _DONT_USE_LOG_UART_TX_INT_
UINT8 uart_print_buf(void)
{
#ifdef ENABLE_LOGGER
    UINT8 i;
    UINT8 wlen;
    UINT16 buf_rptr;        

    DEF_CRITICAL_SECTION_STORAGE;

    if (lbm_queue.length == 0)
    {
        return FALSE;
    }

    wlen = UART_FIFO_MAX_LENGTH - UART_READ(UART_TFL_OFF);        

    if (wlen < 14)
    {
        return TRUE;
    }

    MINT_OS_ENTER_CRITICAL();
    if (wlen > lbm_queue.length)
    {
        wlen = lbm_queue.length;
    }
    buf_rptr = lbm_queue.rd_ptr;

    lbm_queue.length -= wlen;
    if (lbm_queue.length == 0)
    {
        /* log buffer empty, we can reset the read/write pointer */
        lbm_queue.rd_ptr = 0;
        lbm_queue.wr_ptr = 0;
    }
    else
    {
        lbm_queue.rd_ptr = (buf_rptr + wlen) & (LBM_BUFFER_SIZE - 1);    
    }    
    MINT_OS_EXIT_CRITICAL();    

    // Load the data into the UART1 fifo
    for(i = 0; i < wlen; i++)
    {
        // push data to UART FIFO
        UART_WRITE(UART_SEND_BUF_OFF, lbm_queue.buf[buf_rptr]);  
        buf_rptr = (buf_rptr + 1) & (LBM_BUFFER_SIZE - 1);
    }   
#endif
    return TRUE;
}
#endif

//#endif // ENABLE_LOGGER || DIRECT_UART_PRINT
