/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

#ifndef __BT_FW_CONFIG_H__
#define __BT_FW_CONFIG_H__

#include "bz_config.h"
#include "bt_fw_globals.h"
#ifdef USE_FREERTOS
#include "heap_imp.h"
#endif

#define BT_FW_STK_APPL_MEM_POOL_REQ  0x0
#define BT_FW_STK_APPL_HEAP_REQ  0x0

#define LMP_MAX_INQUIRY_RESULT_DATA      4 /* Make sure that this is 2^n */

#define LMP_MAX_ACTIVE_CONN_ENTITIES     7

#define LMP_MAX_PARKED_CONN_ENTITIES     1

/* Maximum number of piconets for the device in scatternet */
#define LMP_MAX_PICONETS_SUPPORTED       4

#define LMP_MAX_CE_DATABASE_ENTRIES     (     \
	LMP_MAX_ACTIVE_CONN_ENTITIES +     \
	LMP_MAX_PICONETS_SUPPORTED - 1)

/* One for ASB and one for PSB. */
#define LMP_MAX_BC_CONNECTIONS 2

#define LMP_MAX_CONN_HANDLES            (     \
           LMP_MAX_CE_DATABASE_ENTRIES +      \
           LMP_MAX_SCO_CONNECTIONS +          \
           LMP_MAX_BC_CONNECTIONS)


/* Maximum number of sco connections supported */
#ifdef SCO_OVER_HCI
#define LMP_MAX_SCO_CONNECTIONS          3
#else /* SCO_OVER_HCI */
#define LMP_MAX_SCO_CONNECTIONS          2
#endif /* SCO_OVER_HCI */

/* The additional entry is used for SCO change connection packet type
 * negotiation.
 */
#define LMP_MAX_SCO_CONN_ENTRIES        (LMP_MAX_SCO_CONNECTIONS+1)

#define HOLD_REQ_INSTANT_MULTIPLIER      9

#define HOLD_INSTANT_MULTIPLIER         10

#define LMP_DELTA_BEACON                 0
#define LMP_DBEACON                      2
#define LMP_TBEACON                      0
#define LMP_NBEACON                      0
#define LMP_DELTA_BEACON                 0

/*
 * Max slot supported by the local device.
 */
#define  LMP_MAX_SLOT                    5

/* LC module defines */
#define LC_MAX_RETRY_COUNT            0x40
#define LC_MAX_NUMBER_OF_HEADERS       (0x10) /* Make sure this is 2^n */
#define LC_MAX_CMD_QUEUE_SIZE           4

/* HCI Command packet size */
/* Restrict the overall size to be maximum 0xFF Bytes (1 Byte
 * size of the command parameters, excluding event header)
 * The buffer size has to be a multiple of 4, because of the
 * AMX memory allocation restrictions.
 */
#define HCI_CMD_BUFFER_SIZE          260  /* 256 CONSIDER CMD HEADER */

/* Event packet size */
/*
 * Restrict the overall event size to be maximum 0xFF Bytes (1 Byte
 * size of the event parameters, excluding event header). The buffer
 * size has to be a multiple of 4, because of the AMX memory allocation 
 * restrictions.
 */
#define HCI_EVENT_BUFFER_SIZE          260 /* CONSIDER EVENT HEADER */


/* ACL Data size of Host from Controller direction*/
/*
 * The buffer size has to be a multiple of 4, because of the 
 * AMX memory allocation restrictions.
 */
#define HCI_ACL_RX_DATA_PAYLOAD_SIZE       1024

/* ACL Data size of Host to Controller direction*/
/*
 * The buffer size has to be a multiple of 4, because of the 
 * AMX memory allocation restrictions.
 */
#define HCI_ACL_DATA_PAYLOAD_SIZE   HCI_ACL_DATA_REPORT_SIZE

#define HCI_ACL_DATA_PKT_SIZE   (((HCI_ACL_DATA_REPORT_SIZE + 7) & 0xFFF8)+24)

#ifdef ENABLE_LOGGER
#define HCI_LOG_DATA_PAYLOAD_SIZE       50
#endif

/*
 * To be used for ACL data transmission from host controller
 * to host direction.
 */
#define HC_TO_HOST_ACL_DATA_PKT_SIZE  (HCI_ACL_RX_DATA_PAYLOAD_SIZE + 8)

/*
 * SCO data size
 */

// revised from 244 to 260 by guochunxia 20090414
#ifdef COMPILE_ESCO 
#define HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE       255 // 240
#define HCI_SYNCHRONOUS_DATA_PKT_SIZE           260 // 244
#elif defined(SCO_OVER_HCI)
#define HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE        144 //90  // modified from 90/94 to 144/148
#define HCI_SYNCHRONOUS_DATA_PKT_SIZE            148 //94
#else
#define HCI_SYNCHRONOUS_DATA_PAYLOAD_SIZE        0
#define HCI_SYNCHRONOUS_DATA_PKT_SIZE            0
#endif /* COMPILE_ESCO */

/*
*  sco link list entry for down link
*/
#define SYNCHRONOUS_ENTRY_NUMBER                 16
#define SYNCHRONOUS_ENTRY_SIZE                   32

#define HCI_LOOPBACK_NUM_SCO_LINKS                3


/* 2 byte (am_addr and len) + 17 bytes(payload) +  4 bytes next_pdu +
 * 4 bytes PDU_Q_data + 1 byte reserved_01 + 2 bytes ce_index +
 * 2 bytes reserved_02 : Refer LMP_PDU_PKT
 * Note: LMP_PDU_BUFFER_SIZE to be updated when there is a change in
 * LMP_PDU_PKT data structure (lmp_external_defines.h).
 */
#ifdef _DMA_LOOPBACK_TEST_
#define LMP_PDU_BUFFER_SIZE             1
#else
#define LMP_PDU_BUFFER_SIZE             28+4
#endif

#ifdef _DMA_LOOPBACK_TEST_
#define LMP_FHS_PKT_BUFFER_SIZE         1
#else
#define LMP_FHS_PKT_BUFFER_SIZE         24+244
#endif

#define LMP_MAX_BD_TO_CE_TBLS           LMP_MAX_CE_DATABASE_ENTRIES

#define BTC_FEATURES_LEN                 8



/* 
 *  Calculate the heap memory requirement for the BTController.
 */
#if defined (SCO_OVER_HCI) || defined(COMPILE_ESCO)                                
#define BT_FW_ESCO_HEAP_REQ \
    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST* (MEM_POOL_ITEM_SZ*1+HCI_SYNCHRONOUS_DATA_PKT_SIZE))+\
    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST*(MEM_POOL_ITEM_SZ*1+HCI_SYNCHRONOUS_DATA_PKT_SIZE))

#define BT_FW_ESCO_HEAP_ACTUAL_PACKET_BUFFER_REQ    \
    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST*HCI_SYNCHRONOUS_DATA_PKT_SIZE)+\
    (BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST*HCI_SYNCHRONOUS_DATA_PKT_SIZE)


#define BT_FW_ESCO_MEM_POOL_REQ 2
#else 
#define BT_FW_ESCO_HEAP_REQ     0
#define BT_FW_ESCO_HEAP_ACTUAL_PACKET_BUFFER_REQ    0
#define BT_FW_ESCO_MEM_POOL_REQ 0
#endif /* defined(SCO_OVER_HCI) || defined(COMPILE_ESCO) */ 

#ifndef _NEW_HCIDMA_FROM_V1_3_
#define LL_POLL_HCI_MAX_TX_ACL_PKT_CNT      16
#define LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE     64
#define LL_POLL_HCI_MAX_RX_ACL_PKT_CNT      16
#define LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE     64
#else
#define LL_POLL_HCI_MAX_TX_ACL_PKT_CNT      8
#define LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE     264 //hci hdr (4) + max payload (251) + ptr (4) + param(4)
#define LL_POLL_HCI_MAX_RX_ACL_PKT_CNT      8
#define LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE     264   
#endif
#define LL_POLL_CONTROL_PDU_BUFFER_NUM      16
#define LL_POLL_CONTROL_PDU_BUFFER_SIZE     32

#define LL_POLL_HCI_TX_ACL_PKT_OFFSET       0

#ifndef LE_MODE_EN
#define LL_FW_HEAP_REQ                      0
#define LL_FW_HEAP_ACTUAL_PACKET_BUFFER_REQ 0
#else /* else of LE_MODE_EN */

#define LL_POLL_ONE_HCI_TX_ACL_PKT_MEM_COST \
    ((MEM_POOL_ITEM_SZ * 1) + LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE)
#define LL_POLL_ONE_HCI_RX_ACL_PKT_MEM_COST \
    ((MEM_POOL_ITEM_SZ * 1) + LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE)
#define LL_POLL_ONE_CONTROL_PDU_BUFFER_MEM_COST \
    ((MEM_POOL_ITEM_SZ * 1) + LL_POLL_CONTROL_PDU_BUFFER_SIZE)

#define LL_FW_HEAP_REQ \
    ((LL_POLL_ONE_HCI_TX_ACL_PKT_MEM_COST * LL_POLL_HCI_MAX_TX_ACL_PKT_CNT) + \
    (LL_POLL_ONE_HCI_RX_ACL_PKT_MEM_COST * LL_POLL_HCI_MAX_RX_ACL_PKT_CNT) + \
    (LL_POLL_ONE_CONTROL_PDU_BUFFER_MEM_COST * LL_POLL_CONTROL_PDU_BUFFER_NUM))

#define LL_FW_HEAP_ACTUAL_PACKET_BUFFER_REQ \
    ((LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE * LL_POLL_HCI_MAX_TX_ACL_PKT_CNT) + \
    (LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE * LL_POLL_HCI_MAX_RX_ACL_PKT_CNT) + \
    (LL_POLL_CONTROL_PDU_BUFFER_SIZE * LL_POLL_CONTROL_PDU_BUFFER_NUM))
  
#endif /* end of LE_MODE_EN */

                    
/*---------------------------------------------------------------------
	Chris add for USB_DMA 2010.03.23
-----------------------------------------------------------------------*/
#define ACL_RX_DES_NUM		17 /* modified by austin for the concern of 
                                  dual mode and log packet to host */

#define SCO_RX_DES_NUM		17
#define EVT_RX_DES_NUM		9

#define DMA_RX_ENTRY_NUM	\
(ACL_RX_DES_NUM)+		\
(SCO_RX_DES_NUM)+		\
(EVT_RX_DES_NUM)

#ifdef LE_MODE_EN
#define DMA_TX_ENTRY_NUM  	(BT_FW_TOTAL_ACL_PKTS_FROM_HOST)+(BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST)+\
							(BT_FW_CMD_BUFFERS)+(LL_POLL_HCI_MAX_TX_ACL_PKT_CNT)
#else
#define DMA_TX_ENTRY_NUM  	(BT_FW_TOTAL_ACL_PKTS_FROM_HOST)+(BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST)+\
							(BT_FW_CMD_BUFFERS)
#endif
/*
#define USB_DMA_RX_REQ		\
		((ACL_RX_DES_NUM + SCO_RX_DES_NUM + EVT_RX_DES_NUM)*4)
*/
#define DMA_RESERVED            1024      
#define DMA_RX_ENTRY_SIZE       8
#define DMA_TX_ENTRY_SIZE       4
//for 4 byte alignment
#define FOUR_BYTES_ALIGN_COMPENDATE_SIZE  3
//for 8 byte alignment
#define EIGHT_BYTES_ALIGN_COMPENDATE_SIZE  7


#define FOUR_BYTE_ALIGNMENT     4
#define EIGHT_BYTE_ALIGNMENT    8

/*
 * We had used the actual hash define for the size of the packets rather than 
 * the variable from the bt_config as these are not supposed to be confiurable 
 * from EEPROM.
 */
     /* (austin)
        1. do not reserved too much MEM_POOL_ITEM_SZ 
        2. do not use and mallow the buffer of LMP_MAX_BD_TO_CE_TBLS
        3. do not mallow the buffer of dma rx desc entry here
        4. do not mallow the buffer of dma tx desc entry here */
#define BT_FW_CONTROLLER_HEAP_REQ                                          \
    (BT_FW_TOTAL_ACL_PKTS_FROM_HOST *(MEM_POOL_ITEM_SZ*1+HCI_ACL_DATA_PKT_SIZE))+ \
    (BT_FW_PDU_BUFFERS*(MEM_POOL_ITEM_SZ*1+LMP_PDU_BUFFER_SIZE))+             \
    (BT_FW_CMD_BUFFERS*(MEM_POOL_ITEM_SZ*1+HCI_CMD_BUFFER_SIZE))+             \
    (BT_FW_TOTAL_ACL_PKTS_TO_HOST*(MEM_POOL_ITEM_SZ*1+HC_TO_HOST_ACL_DATA_PKT_SIZE))+ \
    (BT_FW_EVENT_BUFFERS*(MEM_POOL_ITEM_SZ*1+HCI_EVENT_BUFFER_SIZE))+         \
    (BT_FW_FHS_PKT_BUFFERS*(MEM_POOL_ITEM_SZ*1+LMP_FHS_PKT_BUFFER_SIZE))+     \
    (FOUR_BYTES_ALIGN_COMPENDATE_SIZE*4)+                                    \
    (EIGHT_BYTES_ALIGN_COMPENDATE_SIZE*4)+                                   \
    (BT_FW_ESCO_HEAP_REQ)+(LL_FW_HEAP_REQ) 

#define BT_FW_CONTROLLER_HEAP_ACTUAL_PACKET_BUFFER_REQ          \
    (BT_FW_TOTAL_ACL_PKTS_FROM_HOST *(HCI_ACL_DATA_PKT_SIZE))+ \
    (BT_FW_PDU_BUFFERS*(LMP_PDU_BUFFER_SIZE))+             \
    (BT_FW_CMD_BUFFERS*(HCI_CMD_BUFFER_SIZE))+             \
    (BT_FW_TOTAL_ACL_PKTS_TO_HOST*(HC_TO_HOST_ACL_DATA_PKT_SIZE))+ \
    (BT_FW_EVENT_BUFFERS*(HCI_EVENT_BUFFER_SIZE))+         \
    (BT_FW_FHS_PKT_BUFFERS*(LMP_FHS_PKT_BUFFER_SIZE))+     \
    (FOUR_BYTES_ALIGN_COMPENDATE_SIZE*4)+                                    \
    (EIGHT_BYTES_ALIGN_COMPENDATE_SIZE*4)+                                   \
    (BT_FW_ESCO_HEAP_ACTUAL_PACKET_BUFFER_REQ)+(LL_FW_HEAP_ACTUAL_PACKET_BUFFER_REQ) 

/*
 * Instead of sizeof(LMP_BD_TBL_TO_CE_INDEX_MAP) we had calculated and used 
 * the size i.e 16 bytes.
 */
#define BT_FW_HEAP_RESERVE  0//128

#ifndef _ENABLE_RETENTION_FLOW_FOR_DLPS_
#define OS_MAX_HEAP_MEMORY     \
    ( (BT_FW_HEAP_RESERVE )+ \
     (BT_FW_CONTROLLER_HEAP_ACTUAL_PACKET_BUFFER_REQ) + \
     (BT_FW_STK_APPL_HEAP_REQ) + \
     (DMA_RESERVED))

#define OS_MAX_HEAP_MEMORY_PARTIAL_OFF              0
#else
/* For DLPS mode, we need separate the heap memory in SRAM into Partial On and 
   Off region memory */

#ifdef USE_FREERTOS
#define OS_MALLOC_SIZE(s) MEM_BLOCK_SIZE_ALIGN(MEM_BLOCK_HEADER_SIZE + (s))

#define OS_POOL_STACK_MEM_SIZE(size_, num_, align_, comp_) \
    OS_MALLOC_SIZE(sizeof (OS_ADDRESS) * (num_))
#define OS_POOL_MEM_SIZE(size_, num_, align_, comp_) \
    OS_MALLOC_SIZE((size_) * (num_) + (comp_))


#define OS_POOL_SIZE_HCI_H2C_ACL \
    OS_POOL_SIZE_EXPAND(HCI_ACL_DATA_PKT_SIZE, BT_FW_TOTAL_ACL_PKTS_FROM_HOST, EIGHT_BYTE_ALIGNMENT, EIGHT_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_LMP_PDU \
    OS_POOL_SIZE_EXPAND(LMP_PDU_BUFFER_SIZE, BT_FW_PDU_BUFFERS, FOUR_BYTE_ALIGNMENT, FOUR_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_HCI_CMD \
    OS_POOL_SIZE_EXPAND(HCI_CMD_BUFFER_SIZE, BT_FW_CMD_BUFFERS, FOUR_BYTE_ALIGNMENT, FOUR_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_HCI_EVT \
    OS_POOL_SIZE_EXPAND(HCI_EVENT_BUFFER_SIZE, BT_FW_EVENT_BUFFERS, 0, 0)
#define OS_POOL_SIZE_HCI_C2H_ACL \
    OS_POOL_SIZE_EXPAND(HC_TO_HOST_ACL_DATA_PKT_SIZE, BT_FW_TOTAL_ACL_PKTS_TO_HOST, 0, 0)
#define OS_POOL_SIZE_LMP_FHS_PKT \
    OS_POOL_SIZE_EXPAND(LMP_FHS_PKT_BUFFER_SIZE, BT_FW_FHS_PKT_BUFFERS, 0, 0)
#define OS_POOL_SIZE_HCI_H2C_SCO \
    OS_POOL_SIZE_EXPAND(HCI_SYNCHRONOUS_DATA_PKT_SIZE, BT_FW_TOTAL_SYNCHRONOUS_PKTS_FROM_HOST, FOUR_BYTE_ALIGNMENT, FOUR_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_HCI_C2H_SCO \
    OS_POOL_SIZE_EXPAND(HCI_SYNCHRONOUS_DATA_PKT_SIZE, BT_FW_TOTAL_SYNCHRONOUS_PKTS_TO_HOST, 0, 0)
#define OS_POOL_SIZE_LL_H2C_ACL \
    OS_POOL_SIZE_EXPAND(LL_POLL_HCI_MAX_TX_ACL_PKT_SIZE, LL_POLL_HCI_MAX_TX_ACL_PKT_CNT, EIGHT_BYTE_ALIGNMENT, EIGHT_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_LL_C2H_ACL \
    OS_POOL_SIZE_EXPAND(LL_POLL_HCI_MAX_RX_ACL_PKT_SIZE, LL_POLL_HCI_MAX_RX_ACL_PKT_CNT, EIGHT_BYTE_ALIGNMENT, EIGHT_BYTES_ALIGN_COMPENDATE_SIZE)
#define OS_POOL_SIZE_LL_LLC_PDU \
    OS_POOL_SIZE_EXPAND(LL_POLL_CONTROL_PDU_BUFFER_SIZE, LL_POLL_CONTROL_PDU_BUFFER_NUM, 0, 0)


//#define OS_POOL_SIZE_EXPAND(s, n, a, c) OS_POOL_MEM_SIZE(s, n, a, c)
#define OS_MAX_HEAP_MEMORY OS_POOL_SIZE_HCI_EVT
//#undef OS_POOL_SIZE_EXPAND

//#define OS_POOL_SIZE_EXPAND(s, n, a, c) OS_POOL_MEM_SIZE(s, n, a, c)
#define OS_MAX_HEAP_MEMORY_PARTIAL_OFF \
    (BT_FW_HEAP_RESERVE \
    + OS_POOL_SIZE_HCI_H2C_ACL \
    + OS_POOL_SIZE_LMP_PDU \
    + OS_POOL_SIZE_HCI_CMD \
    + OS_POOL_SIZE_HCI_C2H_ACL \
    + OS_POOL_SIZE_LMP_FHS_PKT \
    + OS_POOL_SIZE_HCI_H2C_SCO \
    + OS_POOL_SIZE_HCI_C2H_SCO \
    + OS_POOL_SIZE_LL_H2C_ACL \
    + OS_POOL_SIZE_LL_C2H_ACL \
    + OS_POOL_SIZE_LL_LLC_PDU \
    + BT_FW_STK_APPL_HEAP_REQ \
    + DMA_RESERVED)
//#undef OS_POOL_SIZE_EXPAND

// TODO: Tune this value
//#define OS_POOL_SIZE_EXPAND(s, n, a, c) (n)
#define OS_MAX_HEAP_DESC_PARTIAL_OFF \
    (OS_POOL_SIZE_HCI_H2C_ACL \
    + OS_POOL_SIZE_LMP_PDU \
    + OS_POOL_SIZE_HCI_CMD \
    + OS_POOL_SIZE_HCI_C2H_ACL \
    + OS_POOL_SIZE_LMP_FHS_PKT \
    + OS_POOL_SIZE_HCI_H2C_SCO \
    + OS_POOL_SIZE_HCI_C2H_SCO \
    + OS_POOL_SIZE_LL_H2C_ACL \
    + OS_POOL_SIZE_LL_C2H_ACL \
    + OS_POOL_SIZE_LL_LLC_PDU \
    + BT_FW_STK_APPL_HEAP_REQ \
    + (DMA_RESERVED / 64))
//#undef OS_POOL_SIZE_EXPAND
#else
#define OS_MAX_HEAP_MEMORY     \
        (BT_FW_EVENT_BUFFERS*(HCI_EVENT_BUFFER_SIZE))

#define OS_MAX_HEAP_MEMORY_PARTIAL_OFF      \
        ( (BT_FW_HEAP_RESERVE )+ \
         (BT_FW_CONTROLLER_HEAP_ACTUAL_PACKET_BUFFER_REQ - OS_MAX_HEAP_MEMORY) + \
         (BT_FW_STK_APPL_HEAP_REQ) + \
         (DMA_RESERVED))
#endif

#endif

#if defined(_NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_) \
    && defined(_SUPPORT_LE_HCI_TX_PKT_FRAGMENT_REASSEMBLE_FUNC_)
#define LL_MISC_ACL_PKT_NODE_ALL_FRAG_INFO_SIZE (sizeof (LL_HCI_ACL_DATA_FRAGMENT_INFO) * LL_MISC_ACL_DATA_PKT_MAX_NODES * LL_MISC_ACL_DATA_PKT_MAX_FRAGS)
#else
#define LL_MISC_ACL_PKT_NODE_ALL_FRAG_INFO_SIZE 0
#endif

#ifdef USE_NEW_LE_SCHEDULER
//#define LE_SCHED_PKT_FIFO_BUF_SIZE OS_MALLOC_SIZE(sizeof (LE_SCHED_PKT_FIFO) * LL_MAX_CONNECTION_UNITS)
//#define LE_SCHED_PKT_FIFO_NODE_BUF_SIZE OS_MALLOC_SIZE(sizeof (LE_SCHED_PKT_FIFO_NODE) * LL_MAX_CONNECTION_UNITS * 8)
#define LE_SCHED_PKT_FIFO_BUF_SIZE 0
#define LE_SCHED_PKT_FIFO_NODE_BUF_SIZE 0
#else
#define LE_SCHED_PKT_FIFO_BUF_SIZE 0
#define LE_SCHED_PKT_FIFO_NODE_BUF_SIZE 0
#endif

#define BT_FW_RESERVED_HEAP_DMEM_MEMORY   (256 \
        + LL_MISC_ACL_PKT_NODE_ALL_FRAG_INFO_SIZE \
        + LE_SCHED_PKT_FIFO_BUF_SIZE + LE_SCHED_PKT_FIFO_NODE_BUF_SIZE)

#ifdef USE_FREERTOS
#define OS_MAX_HEAP_DMEM_MEMORY \
    (OS_POOL_SIZE_HCI_H2C_ACL \
    + OS_POOL_SIZE_LMP_PDU \
    + OS_POOL_SIZE_HCI_CMD \
    + OS_POOL_SIZE_HCI_C2H_ACL\
    + OS_POOL_SIZE_HCI_EVT \
    + OS_POOL_SIZE_LMP_FHS_PKT \
    + OS_POOL_SIZE_HCI_H2C_SCO \
    + OS_POOL_SIZE_HCI_C2H_SCO \
    + OS_POOL_SIZE_LL_H2C_ACL \
    + OS_POOL_SIZE_LL_C2H_ACL \
    + OS_POOL_SIZE_LL_LLC_PDU \
    + BT_FW_RESERVED_HEAP_DMEM_MEMORY)

#define OS_MAX_HEAP_DESC_DMEM_OFF   (1024 / MEM_BLOCK_MIN_SIZE)
#define OS_MAX_HEAP_DMEM_MEMORY_OFF 1024
#else
#define OS_MAX_HEAP_DMEM_MEMORY \
    (((BT_FW_CONTROLLER_HEAP_REQ) - (BT_FW_CONTROLLER_HEAP_ACTUAL_PACKET_BUFFER_REQ)) +\
    ((BT_FW_ESCO_HEAP_REQ) - (BT_FW_ESCO_HEAP_ACTUAL_PACKET_BUFFER_REQ)) +\
    ((LL_FW_HEAP_REQ) - (LL_FW_HEAP_ACTUAL_PACKET_BUFFER_REQ)) +\
    (BT_FW_RESERVED_HEAP_DMEM_MEMORY))
#endif

#ifdef USE_FREERTOS
#define OS_MAX_QUEUE_MEMORY \
    (OS_MALLOC_SIZE( \
            (TIMER_TASK_Q_SIZE \
            + LC_RX_TASK_Q_SIZE \
            + LC_TX_TASK_Q_SIZE \
            + LMP_TASK_Q_SIZE \
            + HCI_CMD_TASK_Q_SIZE \
            + HCI_EVENT_TASK_Q_SIZE \
            + HCI_TD_TASK_Q_SIZE \
            + CH_AS_TASK_Q_SIZE \
            + LOGGER_TASK_Q_SIZE \
            + MAILBOX_TASK_Q_SIZE \
            + ISR_EXTENDED_TASK_Q_SIZE) * OS_Q_ITEM_SIZE))
#else
#define	OS_MAX_QUEUE_MEMORY	BT_FW_TASK_Q_HEAP_REQ
#endif

/* Configurable Parameters  */
#define BT_FW_ACL_MEM_POOL_REQ   		2
#define BT_FW_PDU_MEM_POOL_REQ   		1
#define BT_FW_CMD_MEM_POOL_REQ   		1
#define BT_FW_EVENT_MEM_POOL_REQ 		1
#define BT_FW_FHS_PKT_MEM_POOL_REQ 		1
#define BT_FW_BD_TO_CE_TBL_MEM_POOL_REQ 0  // modified from 1 to 0 by guochunxia
#define BT_FW_DMA_RX_MEM_POOL_REQ       0
#define BT_FW_DMA_TX_MEM_POOL_REQ       0
#define BT_FW_SCO_TX_CONTROL_QUEUE_REQ  1


#ifdef LE_MODE_EN
#define BT_FW_LL_TX_ACL_MEM_POOL_REQ    1
#define BT_FW_LL_RX_ACL_MEM_POOL_REQ    1
#define BT_FW_LL_CTRL_PDU_MEM_POOL_REQ  1
#else
#define BT_FW_LL_TX_ACL_MEM_POOL_REQ    0
#define BT_FW_LL_RX_ACL_MEM_POOL_REQ    0
#define BT_FW_LL_CTRL_PDU_MEM_POOL_REQ  0
#endif

#define BT_FW_CONTROLLER_MEM_POOL_REQ   \
(BT_FW_ACL_MEM_POOL_REQ)+      			\
(BT_FW_PDU_MEM_POOL_REQ)+             	\
(BT_FW_CMD_MEM_POOL_REQ)+             	\
(BT_FW_EVENT_MEM_POOL_REQ)+ 			\
(BT_FW_FHS_PKT_MEM_POOL_REQ)+         	\
(BT_FW_BD_TO_CE_TBL_MEM_POOL_REQ)+     	\
(BT_FW_ESCO_MEM_POOL_REQ)+              	\
(BT_FW_DMA_RX_MEM_POOL_REQ)+            	\
(BT_FW_DMA_TX_MEM_POOL_REQ)+            \
(BT_FW_LL_TX_ACL_MEM_POOL_REQ)+         \
(BT_FW_LL_RX_ACL_MEM_POOL_REQ)+         \
(BT_FW_SCO_TX_CONTROL_QUEUE_REQ)+       \
(BT_FW_LL_CTRL_PDU_MEM_POOL_REQ)

#define BT_FW_MEM_POOL_RESERVE 0

#define     OS_MAX_POOLS  				\
		(BT_FW_MEM_POOL_RESERVE)+		\
		(BT_FW_CONTROLLER_MEM_POOL_REQ) +\
		(BT_FW_STK_APPL_MEM_POOL_REQ)

#endif /* __BT_FW_CONFIG_H__ */
