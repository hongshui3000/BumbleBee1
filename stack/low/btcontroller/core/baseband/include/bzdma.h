/***************************************************************************
 Copyright (C) Realtek
 This module is a confidential and proprietary property of Realtek and
 a possession or use of this module requires written permission of Realtek.
 ***************************************************************************/

/**
 * \file
 *  BZDMA interface.
 *
 * \author 
 *  austin <austin_chen@realtek.com>, (C) 2010
 */

/** \addtogroup bb_driver_bzdma Baseband Driver Module
 *  @{ */
#ifndef __BB_BZDMA_H__
#define __BB_BZDMA_H__

#include "DataType.h"
#include "bt_fw_os.h"

#define BZDMA_USE_DEDICATED_TX_ENTRY  1
#define BZDMA_INTERLACE_CE_DATA_EN    0   /* enable it, bzdma can interlace the
                                            bzdma txcmd for LE data burst 
                                            transfer */
                                            

#define BZDMA_MAX_ACL_PKT_SZ        1021
#define BZDMA_MAX_BCST_PKT_SZ       240
#define BZDMA_MAX_SCO_PKT_SZ        1016

#define BZDMA_TX_DESC_ONE_ENTRY_SEG_NUM     8 /* HW maximum is 16 */

#if defined(_NEW_BZDMA_FROM_V8_) || defined(_NEW_BZDMA_FROM_V7_) 
#define BZDMA_TX_DESC_ENTRY_NUM             13
#else
#define BZDMA_TX_DESC_ENTRY_NUM             9
#endif

#ifdef _NEW_BZDMA_FROM_V8_
#define BZDMA_LEONLY_TX_DESC_ENTRY_NUM     4
#else
#define BZDMA_LEONLY_TX_DESC_ENTRY_NUM     0
#endif

#define BZDMA_RX_DESC_ENTRY_NUM             8

#define BZDMA_DBG_MSG_MAX_CNT               6

#define BZDMA_DEFAULT_TX_FREE_ENTRY_BM      ((1 << BZDMA_TX_DESC_ENTRY_NUM) - 1)
#define BZDMA_DEFAULT_RX_FREE_ENTRY_BM      ((1 << BZDMA_RX_DESC_ENTRY_NUM) - 1)

#ifdef _NEW_BZDMA_FROM_V8_
#define BZDMA_LEONLY_DEFAULT_RX_FREE_ENTRY_BM  ((1 << BZDMA_LEONLY_TX_DESC_ENTRY_NUM) - 1)
#else
#define BZDMA_LEONLY_DEFAULT_RX_FREE_ENTRY_BM   0
#endif

#define BZDMA_RXCMD_INT_EN_MASK      0x0000     /* no rx cmd */

#   ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
#define BZDMA_TXCMD_INT_EN_MASK      0x0003     /* only tx cmd 0, 1 */   
#   else
#define BZDMA_TXCMD_INT_EN_MASK      0x0201     /* only tx cmd 0, 9 */
#   endif

#define BZDMA_CTRL_CONFIG_ORI        0x0001
#define BZDMA_INT_EN_MASK    (BZDMA_RXCMD_INT_EN_MASK | BZDMA_TXCMD_INT_EN_MASK)

#ifdef _NEW_BZDMA_FROM_V8_
#define BZDMA_LEONLY_TXCMD_INT_EN_MASK      0x0000     /* no tx cmd */
#else
#define BZDMA_LEONLY_TXCMD_INT_EN_MASK      0x0000     /* no tx cmd */
#endif
                                     
#if BZDMA_INTERLACE_CE_DATA_EN
#define BZDMA_CTRL_CONFIG           (BZDMA_CTRL_CONFIG_ORI | 0x08)
#else
#define BZDMA_CTRL_CONFIG           BZDMA_CTRL_CONFIG_ORI
#endif

#if defined(_NEW_BZDMA_FROM_V7_) && (defined(BZDMA_USE_NEW_SCO_TXCMD) || defined(_USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_))
#   ifndef _USE_ONE_NEW_BZDMA_TXCMD_FOR_SCO_
#define BZDMA_SCO_TXCMD_INT_MAP     (BIT9 | BIT10 | BIT11)
#   else
#define BZDMA_SCO_TXCMD_INT_MAP     (BIT9)
#   endif
#else
#define BZDMA_SCO_TXCMD_INT_MAP     BIT1
#endif  

#ifndef _NEW_BZDMA_FROM_V7_
#define BZDMA_RXCMD_DONE_INT_BIT    BIT9
#else
#define BZDMA_RXCMD_DONE_INT_BIT    BIT12
#endif

/* BZ DMA register list */
#define BZDMA_REG_BASE              0x40058000
#define BZDMA_REG_TX_CMD(num)       (0x30C - ((num) << 2))
#define BZDMA_REG_CTRL              0x310
#define BZDMA_REG_RX_CMD            0x314
#define BZDMA_REG_RX_CMD1           0x318

#define BZDMA_REG_INT_STATUS        0x31C
#define BZDMA_REG_INT_ENABLE        0x320
#define BZDMA_REG_LB_TRIG_CTRL      0x324
#define BZDMA_REG_ERR_RECOVERY      0x328
#define BZDMA_REG_DBG_PORT_SEL      0x32C
#define BZDMA_REG_ACL_TXFIFO_PTR    0x330
#define BZDMA_REG_SCO_TXFIFO_PTR    0x334
#define BZDMA_REG_BRDCST_TXFIFO_PTR 0x338
#define BZDMA_REG_ACL_RXFIFO_PTR    0x33C
#define BZDMA_REG_SCO_RXFIFO_PTR    0x340

#ifdef _NEW_BZDMA_FROM_V8_
#define BZDMA_REG_BLEONLY_TX_CMD(num)                   (0x400 + ((num) << 2))
#define BZDMA_REG_BLEONLY_TX_DATA_DESC_CTRL             0x410
#define BZDMA_REG_BLEONLY_INT_STATUS                    0x414
#define BZDMA_REG_BLEONLY_INT_ENABLE                    0x418
#define BZDMA_REG_BLEONLY_TXDMA_LB_CTRL                 0x41C
#define BZDMA_REG_BLEONLY_ENTRY_SEGMENT_VALID(num)      (0x440 + ((num) << 2))
#endif

#define RD_U32_BZDMA_REG(offset)        RD_32BIT_IO(BZDMA_REG_BASE, offset)
#define RD_U16_BZDMA_REG(offset)        RD_16BIT_IO(BZDMA_REG_BASE, offset)
#define RD_U8_BZDMA_REG(offset)         RD_8BIT_IO(BZDMA_REG_BASE, offset)
#define WR_U32_BZDMA_REG(offset, val)   WR_32BIT_IO(BZDMA_REG_BASE, offset, val)
#define WR_U16_BZDMA_REG(offset, val)   WR_16BIT_IO(BZDMA_REG_BASE, offset, val)
#define WR_U8_BZDMA_REG(offset, val)    WR_8BIT_IO(BZDMA_REG_BASE, offset, val)

#define BZDMA_TRIG_CLR     WR_U32_BZDMA_REG(BZDMA_REG_DBG_PORT_SEL, RD_U32_BZDMA_REG(BZDMA_REG_DBG_PORT_SEL) & 0x7FFFFFFF)     
#define BZDMA_TRIG_SET     WR_U32_BZDMA_REG(BZDMA_REG_DBG_PORT_SEL, RD_U32_BZDMA_REG(BZDMA_REG_DBG_PORT_SEL)| 0x80000000)    

/* because BB ACL packet type and size are adapted, sometimes we need to use 
   mutilple segment to carry payload body from continuous HCI ACL data pkts to 
   aggregate a BB pkt - austin */
#if 0   
#define BZDMA_UNICAST_ACL_TX_SEG \
    (((1021 + HCI_ACL_DATA_REPORT_SIZE - 1)/ HCI_ACL_DATA_REPORT_SIZE) + 1)
#else
#define BZDMA_UNICAST_ACL_TX_SEG            4
#endif

/* The SW Definition of Tx Entry Type in BZDMA  */
enum BZDMA_TX_ENTRY_TYPE_ {
    BZDMA_TX_ENTRY_TYPE_BROADCAST       = 0,
    BZDMA_TX_ENTRY_TYPE_SCO             = 1,        
    BZDMA_TX_ENTRY_TYPE_PICONET0        = 2,
    BZDMA_TX_ENTRY_TYPE_PICONET1        = 3,
    BZDMA_TX_ENTRY_TYPE_PICONET2        = 4,
    BZDMA_TX_ENTRY_TYPE_PICONET3        = 5,   
    BZDMA_TX_ENTRY_TYPE_LE_ADV          = 6,
    BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST0  = 7,
    BZDMA_TX_ENTRY_TYPE_LE_DATA_BURST1  = 8,
#ifdef _NEW_BZDMA_FROM_V7_    
    BZDMA_TX_ENTRY_TYPE_NEW_SCO0        = 9,
    BZDMA_TX_ENTRY_TYPE_NEW_SCO1        = 10,   
    BZDMA_TX_ENTRY_TYPE_NEW_SCO2        = 11,
#ifdef _SUPPORT_CSB_TRANSMITTER_
    BZDMA_TX_ENTRY_TYPE_CSB             = 11,    
#endif
#ifdef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    BZDMA_TX_ENTRY_TYPE_ACL_SNIFF       = 12,   
#endif    
#endif
};

/* The SW Definition of Maximum Segment Number per Tx Entry in BZDMA  */
enum BZDMA_TX_ENTRY_MAX_SEGS_ {
    BZDMA_TX_ENTRY_MAX_SEGS_BROADCAST       = BZDMA_UNICAST_ACL_TX_SEG,
    BZDMA_TX_ENTRY_MAX_SEGS_SCO             = 4,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET0        = BZDMA_UNICAST_ACL_TX_SEG,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET1        = BZDMA_UNICAST_ACL_TX_SEG,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET2        = BZDMA_UNICAST_ACL_TX_SEG,
    BZDMA_TX_ENTRY_MAX_SEGS_PICONET3        = BZDMA_UNICAST_ACL_TX_SEG,
#ifndef _NEW_BZDMA_FROM_V8_    
    BZDMA_TX_ENTRY_MAX_SEGS_LE_ADV          = 2,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0  = 8,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST1  = 8, 
#else /* eles of #ifndef _NEW_BZDMA_FROM_V8_ */
    BZDMA_TX_ENTRY_MAX_SEGS_LE_ADV          = 0,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0  = 0,
    BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST1  = 0, 
#endif /* end of #ifndef _NEW_BZDMA_FROM_V8_ */

#ifndef _NEW_BZDMA_FROM_V7_
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO0        = 0,
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO1        = 0,    
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO2        = 0,
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SNIFF       = 0,
#else /* eles of #ifndef _NEW_BZDMA_FROM_V7_ */
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO0        = 4,
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO1        = 0,    
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO2        = 1, /* for CSB beacon */
    BZDMA_TX_ENTRY_MAX_SEGS_NEW_SNIFF       = BZDMA_UNICAST_ACL_TX_SEG,      
#endif /* end of #ifndef _NEW_BZDMA_FROM_V7_ */
    BZDMA_TX_ENTRY_TOTAL_SEGS = (BZDMA_TX_ENTRY_MAX_SEGS_BROADCAST +
                                 BZDMA_TX_ENTRY_MAX_SEGS_SCO +
                                 BZDMA_TX_ENTRY_MAX_SEGS_PICONET0 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_PICONET1 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_PICONET2 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_PICONET3 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_LE_ADV +
                                 BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST0 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_LE_DATA_BURST1 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO0 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO1 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_NEW_SCO2 +
                                 BZDMA_TX_ENTRY_MAX_SEGS_NEW_SNIFF)
};

#ifdef _NEW_BZDMA_FROM_V8_
/* The SW Definition of Ble Only Tx Entry Type in BZDMA  */
enum BZDMA_LEONLY_TX_ENTRY_TYPE_ {
    BZDMA_LEONLY_TX_ENTRY_TYPE_ADV0        = 0,
    BZDMA_LEONLY_TX_ENTRY_TYPE_ADV1        = 1,        
    BZDMA_LEONLY_TX_ENTRY_TYPE_ADV2        = 2,
    BZDMA_LEONLY_TX_ENTRY_TYPE_DATA        = 3,
};

#define BZDMA_LEONLY_TX_MAX_ENTRY_NUM             8
#define BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS_GRADE  3
#define BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS        (1 << BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS_GRADE)

/* The SW Definition of Maximum Segment Number per Tx Entry in BZDMA  */
enum BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ {
    BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 = 2,
    BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV1 = 0,
    BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV2 = 0,
    BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_DATA = BZDMA_LEONLY_TX_MAX_ENTRY_NUM *
                                          BZDMA_LEONLY_TX_ONE_ENTRY_MAX_SEGS,
};


#define BZDMA_LEONLY_TX_ENTRY_TOTAL_SEGS \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 + \
     BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV1 + \
     BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV2 + \
     BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_DATA)

#define BZDMA_LEONLY_TX_ENTRY_TOTAL_ADV_SEGS \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 + \
     BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV1 + \
     BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV2)

/* The SW Definition of Maximum Fragment Number per Tx Entry in BZDMA  */
enum BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ {
    BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0 = 2,
    BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV1 = 0,
    BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV2 = 0,
    BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_DATA = 2,
};

#define BZDMA_LEONLY_TX_ENTRY_TOTAL_FRAGS \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0) + \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV1 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV1) + \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV2 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV2) + \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_DATA * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_DATA)

#define BZDMA_LEONLY_TX_ENTRY_TOTAL_ADV_FRAGS \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV0 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV0) + \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV1 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV1) + \
    (BZDMA_LEONLY_TX_ENTRY_MAX_SEGS_ADV2 * BZDMA_LEONLY_TX_ENTRY_MAX_FRAGS_ADV2)
#endif


/* The HW Definition of Tx/Rx PID  */
enum BZDMA_PID_ {
    BZDMA_TX_PID_ACL        = 0,
#ifdef _SUPPORT_CSB_TRANSMITTER_  
    BZDMA_TX_PID_CSB        = 1,
#endif    
    BZDMA_TX_PID_BCST       = 2,
    BZDMA_TX_PID_SCO        = 3,
#ifdef _NEW_BZDMA_FROM_V7_  
    BZDMA_TX_PID_NEW_SCO    = 4,    
#endif
    BZDMA_TX_PID_TOTAL      = 3,
    BZDMA_RX_PID_ACL        = 0,
    BZDMA_RX_PID_SCO        = 1,
    BZDMA_RX_PID_TOTAL      = (BZDMA_RX_PID_SCO + 1),
};

/* The HW Definition of LE PDU Type */
enum BZDMA_LE_PDU_TYPE_ {
    BZDMA_LE_PDU_TYPE_DATA                  = 0,
    BZDMA_LE_PDU_TYPE_ADV_DATA_ONLY         = 1,
    BZDMA_LE_PDU_TYPE_SCAN_DATA_ONLY        = 2,    
    BZDMA_LE_PDU_TYPE_ADV_SCAN_DATA_BOTH    = 3,      
    BZDMA_LE_PDU_TYPE_MAX = BZDMA_LE_PDU_TYPE_ADV_SCAN_DATA_BOTH,
};


/* The HW Definition of TRX PCM Format Conversion Type  */
enum BZDMA_CODEC_TRX_PCM_TYPE_ {
    BZDMA_CODEC_PCM_TYPE_2S,
    BZDMA_CODEC_PCM_TYPE_1S,   
    BZDMA_CODEC_PCM_TYPE_SIGN,    
    BZDMA_CODEC_PCM_TYPE_UNSIGN,
    BZDMA_CODEC_PCM_TYPE_INVALID,
};

/* The HW Definition of TRX Codec Conversion Type  */
enum BZDMA_CODEC_TRX_CONV_TYPE_ {
    BZDMA_CODEC_TX_CONV_TYPE_BYPASS = 0,
    BZDMA_CODEC_TX_CONV_TYPE_L2U,
    BZDMA_CODEC_TX_CONV_TYPE_L2A,
    BZDMA_CODEC_TX_CONV_TYPE_A2U,
    BZDMA_CODEC_TX_CONV_TYPE_U2A,
    BZDMA_CODEC_TX_CONV_TYPE_L2C,
    BZDMA_CODEC_TX_CONV_TYPE_A2C,
    BZDMA_CODEC_TX_CONV_TYPE_U2C,
    BZDMA_CODEC_RX_CONV_TYPE_BYPASS = 0,
    BZDMA_CODEC_RX_CONV_TYPE_U2L,
    BZDMA_CODEC_RX_CONV_TYPE_A2L,
    BZDMA_CODEC_RX_CONV_TYPE_U2A,
    BZDMA_CODEC_RX_CONV_TYPE_A2U,
    BZDMA_CODEC_RX_CONV_TYPE_C2L,
    BZDMA_CODEC_RX_CONV_TYPE_C2A,
    BZDMA_CODEC_RX_CONV_TYPE_C2U
};

/* ============ BZ DMA Register Structure (Begin) =================== */
#ifdef _NEW_BZDMA_FROM_V8_
/* The structure of BZDMA_REG_BLEONLY_TX_CMD. This register is used to copy data
   from SRAM to specific Tx FIFO of Bluewiz */
typedef struct BZDMA_REG_S_BLEONLY_TX_CMD_ {
    UINT32 desc_addr:16; /* bit[15:0], the descriptor base address of segment (RW)
                            note: this field must be 8 byte alignment */
    UINT32 rsvd1:6;      /* bit[21:16], reserved field */
    UINT32 le_cmd:2;     /* bit[23:22], PDU attribute of LE TxDMA */
    UINT32 rsvd2:5;      /* bit[28:24], reserved field */
    UINT32 adv_ch_id:2;  /* bit[30:29], advertising channel id */
    UINT32 req:1;        /* bit[31], the Txreq bit for cmd entry
                            FW can set 1 to send the request,
                            then HW will clear it when finish the cmd (RW) */
} BZDMA_REG_S_BLEONLY_TX_CMD, *PBZDMA_REG_S_BLEONLY_TX_CMD;

/* The structure of BZDMA_REG_BLEONLY_TX_DATA_DESC_CTRL. 
   This register is used to set the segment numbers of one data channel entry */
typedef struct BZDMA_REG_S_BLEONLY_TX_DATA_DESC_CTRL_ {
    UINT32 seg_num:3;    /* bit[2:0], Set for segment number of per data channel 
                                      entry. (RW) */
    UINT32 rsvd:29;      /* bit[21:16], reserved field */
} BZDMA_REG_S_BLEONLY_TX_DATA_DESC_CTRL, *PBZDMA_REG_S_BLEONLY_TX_DATA_DESC_CTRL;

/* The structure of BZDMA_REG_BLEONLY_INT_STATUS. This register is used by HW to 
     indicate the interrupt status */
typedef struct BZDMA_REG_S_BLEONLY_INT_STATUS_ {
    UINT32 bmtx_finish_int:4; /* bit[3:0], the bitmap to indicate the 
                                           txdma cmd [3:0] finish int status
                                           (R/W1C) */
    UINT32 rsvd1:12;          /* bit[15:4], reserved */
    UINT32 bmtx_busy:4;       /* bit[19:16], the bitmap to indicate which txdma
                                            cmd [3:0] are busy (R) */
    UINT32 rsvd2:12;          /* bit[31:20], reserved */
} BZDMA_REG_S_BLEONLY_INT_STATUS, *PBZDMA_REG_S_BLEONLY_INT_STATUS;

/* The structure of BZDMA_REG_BLEONLY_INT_ENABLE. This register is used by FW to 
     enable the interrupt */
typedef struct BZDMA_REG_S_BLEONLY_INT_ENABLE_ {
    UINT32 bmtx_finish_int:4; /* bit[3:0], the bitmap to enable the 
                                           txdma cmd [3:0] finish int 
                                           (R/W) */
    UINT32 rsvd:28;           /* bit[31:4], reserved */
} BZDMA_REG_S_BLEONLY_INT_ENABLE, *PBZDMA_REG_S_BLEONLY_INT_ENABLE;

/* The structure of BZDMA_REG_BLEONLY_TXDMA_LB_CTRL. This register is used by FW to 
     emulate hw scheduler behavior */
typedef struct BZDMA_REG_S_BLEONLY_TXDMA_LB_CTRL_ {
    UINT32 flag_lb:1;        /* bit[0], Bleonly_flag when FW drive BLEONLY TXDMA (R/W) */
    UINT32 entry_idx_lb:3;   /* bit[3:1], Bleonly_entry_idx[2:0] when FW drive BLEONLY TXDMA (R/W) */
    UINT32 seg_idx_lb:4;     /* bit[7:4], Bleonly_seg_idx[3:0] when FW drive BLEONLY TXDMA (R/W) */
    UINT32 rsvd1:8;          /* bit[15:8], reserved */
    UINT32 pkt_len:11;       /* bit[26:16], Bleonly Pkt Len (R) */
    UINT32 le_md:1;          /* bit[27], Bleonly More data bit (R) */
    UINT32 le_llid:2;        /* bit[29:28], Bleonly LLID (R) */
    UINT32 rsvd2:2;          /* bit[31:30], reserved */
} BZDMA_REG_S_BLEONLY_TXDMA_LB_CTRL, *PBZDMA_REG_S_BLEONLY_TXDMA_LB_CTRL;
#endif

/* The structure of BZDMA_REG_TX_CMD. This register is used to copy data from
     SRAM to specific Tx FIFO of Mindtree */
typedef struct BZDMA_REG_S_TX_CMD_ {
    UINT32 desc_addr:16; /* bit[15:0], the descriptor base address (RW) */
#ifndef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    UINT32 conn_entry:6; /* bit[21:16], connection entry (RW) */
#else
    UINT32 conn_entry:5; /* bit[20:16], connection entry (RW) */
    UINT32 sniff_en:1;   /* bit[21], the tx command is used for sniff (1) or 
                            non-sniff (0) for acl hw scheduler (RW) */
#endif
    UINT32 le_cmd:2;     /* bit[23:22], PDU attribute of LE TxDMA */
    UINT32 le_en:1;      /* bit[24], Enable LE TxDMA command */
    UINT32 piconet_id:2; /* bit[26:25], the piconet ID */
    UINT32 chid:2;       /* bit[28:27], the SCO channel id for cmd entry (RW) */
    UINT32 pktid:2;      /* bit[30:29], the pkt id for cmd entry (RW) */    
    UINT32 req:1;        /* bit[31], the Txreq bit for cmd entry 
                            FW can set 1 to send the request, 
                            then HW will clear it when finish the cmd (RW) */
} BZDMA_REG_S_TX_CMD, *PBZDMA_REG_S_TX_CMD;

/* The structure of BZDMA_REG_CTRL. This register is used by FW to control
    the settings of BZDMA */
typedef struct BZDMA_REG_S_CTRL_ {
    UINT32 bzdma_en:1;    /* bit[0], set 1/0 to enable/disable bzdma (RW) */
    UINT32 loopback_en:1; /* bit[1], set 1 to enable the bzdma loopback.
                             tx fifo read and rx fifo write via CPU (RW) */
    UINT32 sw_reset:1;    /* bit[2], sw reset for bzdma, high active (RW) */  
    UINT32 no_atomic_ce:1; /* bit[3], CE data transfer is not atomic (RW) */                            
#ifndef _NEW_BZDMA_FROM_V7_
    UINT32 rsvd:28;        /* bit[31:4], reserved */
#else
    UINT32 rsvd:26;        /* bit[29:4], reserved */    
    UINT32 rx_empty:1;     /* bit[30], rxdma cmd fifo empty status (R) */     
    UINT32 rx_full:1;      /* bit[31], rxdma cmd fifo full status (R) */  
#endif
} BZDMA_REG_S_CTRL, *PBZDMA_REG_S_CTRL;

/* The structure of BZDMA_REG_RX_CMD. This register is used to move data from
     specific Rx FIFO of Mindtree to SRAM */
typedef struct BZDMA_REG_S_RX_CMD_ {
    UINT32 chid:2;       /* bit[1:0], SCO channel Id */
    UINT32 pktid:2;      /* bit[3:2], packet id */
    UINT32 len:11;       /* bit[14:4], the rxdma request length in bytes */
    UINT32 start:1;      /* bit[15], request HW to start new rxdma request cmd */
    UINT32 dest_addr:16; /* bit[31:16], The rx dma destination address */
} BZDMA_REG_S_RX_CMD, *PBZDMA_REG_S_RX_CMD;

/* The structure of BZDMA_REG_RX_CMD1. This register is used to move data from
     specific Rx FIFO of Mindtree to SRAM */
typedef struct BZDMA_REG_S_RX_CMD1_ {
    UINT32 cmd_id:3;     /* bit[2:0], rxdma command id */
    UINT32 no_wr_sram:1; /* bit[3], don't write to SRAM */
    UINT32 rsvd:28;      /* bit[14:4], reserved */
} BZDMA_REG_S_RX_CMD1, *PBZDMA_REG_S_RX_CMD1;

/* The structure of BZDMA_REG_INT_STATUS. This register is used by HW to 
     indicate the interrupt status */
typedef struct BZDMA_REG_S_INT_STATUS_ {
#ifndef _NEW_BZDMA_FROM_V7_
    UINT32 bmtx_finish_int:9; /* bit[8:0], the bitmap to indicate the 
                                           txdma cmd [8:0] finish int status 
                                           (R/W1C) */
    UINT32 rx_finish_int:1;  /* bit[9], rxdma cmd finish int status (R/W1C) */     
    UINT32 rx_empty_int:1;   /* bit[10], rxdma cmd fifo empty int status (R/W1C) */     
    UINT32 rx_full_int:1;    /* bit[11], rxdma cmd fifo full int status (R/W1C) */  
    UINT32 rsvd:5;           /* bit[16:12], reserved */
    UINT32 rx_empty:1;       /* bit[17], rxdma cmd fifo empty status (R) */     
    UINT32 rx_full:1;        /* bit[18], rxdma cmd fifo full status (R) */  
    UINT32 rxcmd_last_idx:3; /* bit[21:19], the last rxdma cmd fifo index (R) */ 
    UINT32 bmtx_busy:9;      /* bit[30:22], the bitmap to indicate which txdma 
                                            cmd [8:0] are busy (R) */    
#else
    UINT32 bmtx_finish_int:12; /* bit[11:0], the bitmap to indicate the 
                                           txdma cmd [11:0] finish int status 
                                           (R/W1C) */
    UINT32 rx_finish_int:1;  /* bit[12], rxdma cmd finish int status (R/W1C) */     
    UINT32 rx_empty_int:1;   /* bit[13], rxdma cmd fifo empty int status (R/W1C) */     
    UINT32 rx_full_int:1;    /* bit[14], rxdma cmd fifo full int status (R/W1C) */  
#ifndef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    UINT32 rsvd:1;           /* bit[15], reserved */
#else
    UINT32 bmtx_finish_int_ex:1; /* bit[15], the bitmap to indicate the 
                                           txdma cmd [12] finish int status 
                                           (R/W1C) */
#endif
    UINT32 rxcmd_last_idx:3; /* bit[18:16], the last rxdma cmd fifo index (R) */ 
    UINT32 bmtx_busy:12;     /* bit[30:19], the bitmap to indicate which txdma 
                                            cmd [11:0] are busy (R) */     
#endif
    UINT32 rx_busy:1;        /* bit[31], rxdma cmd is busy (R) */
} BZDMA_REG_S_INT_STATUS, *PBZDMA_REG_S_INT_STATUS;

/* The structure of BZDMA_REG_INT_ENABLE. This register is used by FW to 
     enable the interrupt */
typedef struct BZDMA_REG_S_INT_ENABLE_ {
#ifndef _NEW_BZDMA_FROM_V7_
    UINT32 bmtx_finish_int:9;/* bit[8:0], txdma cmd [8:0] finish INT enable 
                                          bitmap (R/W)*/
    UINT32 rx_finish_int:1;  /* bit[9], rxdma cmd finish INT enable (R/W)*/ 
    UINT32 rx_empty_int:1;   /* bit[10], txdma cmd empty INT enable (R/W)*/
    UINT32 rx_full_int:1;    /* bit[11], rxdma cmd full INT enable (R/W)*/
    UINT32 rsvd:20;          /* bit[31:12], reserved */
#else
    UINT32 bmtx_finish_int:12;/* bit[11:0], txdma cmd [11:0] finish INT enable 
                                          bitmap (R/W)*/
    UINT32 rx_finish_int:1;  /* bit[12], rxdma cmd finish INT enable (R/W)*/ 
    UINT32 rx_empty_int:1;   /* bit[13], txdma cmd empty INT enable (R/W)*/
    UINT32 rx_full_int:1;    /* bit[14], rxdma cmd full INT enable (R/W)*/
#ifndef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    UINT32 rsvd:17;          /* bit[31:15], reserved */
#else
    UINT32 bmtx_finish_int_ex:1;/* bit[15], txdma cmd [12] finish INT enable 
                                          bitmap (R/W)*/
    UINT32 rsvd:16;          /* bit[31:16], reserved */
#endif
#endif
} BZDMA_REG_S_INT_ENABLE, *PBZDMA_REG_S_INT_ENABLE;

/* The structure of BZDMA_REG_TX_TRIG_CTRL. This register is used by FW to 
     trigger the tx comand */
typedef struct BZDMA_REG_S_TX_TRIG_CTRL_ {
    UINT32 le_llid:2;        /* bit[1:0], LE LLID (R)*/
    UINT32 le_more_data:1;   /* bit[2], LE more data bit (R)*/
    UINT32 rsvd1:5;          /* bit[7:3], reserved */
    UINT32 le_adv_scan_id:1; /* bit[8], LE adv or scan response pdu id (R/W)*/
    UINT32 txcmd_id:4;       /* bit[12:9], txdma cmd id (R/W)*/ 
    UINT32 txcmd_req:1;      /* bit[13], txdma cmd request (R/W)*/
    UINT32 txcmd_vld_clr:1;  /* bit[14], txdma cmd valid clear, wrire 1 then 0 
                                         to clear it (R/W)*/
    UINT32 rsvd2:1;          /* bit[15], reserved */ 
    UINT32 pkt_len:11;       /* bit[26:16], packet length (R) */
    UINT32 rsvd3:5;          /* bit[31:27], reserved */   
} BZDMA_REG_S_TX_TRIG_CTRL, *PBZDMA_REG_S_TX_TRIG_CTRL;

/* The structure of BZDMA_REG_ERR_RECOVERY. This register is used by FW to 
     trigger the tx comand */
typedef struct BZDMA_REG_S_ERR_RECOVER_ {
    UINT32 txdma_flush:1;   /* bit[0], FW assert it to flush txdma until 
                               txdma idle bit is true, then deassert it (R/W)*/
    UINT32 rxdma_flush:1;   /* bit[1], FW assert it to flush rxdma until 
                               rxdma idle bit is true, then deassert it (R/W)*/
    UINT32 rsvd1:6;         /* bit[7:2], reserved */
    UINT32 txdma_idle:1;    /* bit[8], txdma state is idle (R) */
    UINT32 rxdma_idle:1;    /* bit[9], rxdma state is idle (R)*/ 
    UINT32 rsvd2:22;        /* bit[31:10], reserved */   
} BZDMA_REG_S_ERR_RECOVER, *PBZDMA_REG_S_ERR_RECOVER;

/* The union of BZDMA_REG. */
typedef union BZDMA_REG_S_ {
    BZDMA_REG_S_TX_CMD TxCmd;
    BZDMA_REG_S_CTRL Ctrl;
    BZDMA_REG_S_RX_CMD RxCmd;
    BZDMA_REG_S_RX_CMD1 RxCmd1;
    BZDMA_REG_S_INT_STATUS IntStatus;
    BZDMA_REG_S_INT_ENABLE IntEnable;
    BZDMA_REG_S_TX_TRIG_CTRL TxScheCtrl;
    BZDMA_REG_S_ERR_RECOVER ErrRecover;
#ifdef _NEW_BZDMA_FROM_V8_
    BZDMA_REG_S_BLEONLY_TX_CMD BleTxCmd;
    BZDMA_REG_S_BLEONLY_TX_DATA_DESC_CTRL BleTxDesCtrl;
    BZDMA_REG_S_BLEONLY_INT_STATUS BleIntStatus;
    BZDMA_REG_S_BLEONLY_INT_ENABLE BleIntEnable;
    BZDMA_REG_S_BLEONLY_TXDMA_LB_CTRL BleTxLbCtrl;
#endif
    UINT32 DWord;
} BZDMA_REG_S, *PBZDMA_REG_S;

/* ============= BZ DMA Register Structure (End) =================== */
/* The structure of one segment of TX Descriptor */
typedef struct BZDMA_TX_DESC_SEGMENT_ {
    union {
        struct {
            UINT32 start_addr:16;   /* bit[15:0], the start address of SRAM */
            UINT32 rsvd0:2;         /* bit[17:16], reserved */
            UINT32 len:11;          /* bit[28:18], the copied length*/
            UINT32 rsvd1:3;         /* bit[31:29], reserved */
            };
        UINT32 DWord0;
        };    
    union {
        struct {
            UINT32 rsvd2:29;        /* bit[60:32], reserved */
            UINT32 llid:2;          /* bit[62:61], the link layer id */
            UINT32 isLast:1;        /* bit[63], is last segment */
            };
        UINT32 DWord1;
        };
} BZDMA_TX_DESC_SEGMENT, *PBZDMA_TX_DESC_SEGMENT;
 
/* The structure of Tx Descriptor of BZDMA */
typedef struct BZDMA_TX_DESC_SET_ {    
    BZDMA_TX_DESC_SEGMENT seg[BZDMA_TX_ENTRY_TOTAL_SEGS];
} BZDMA_TX_DESC_SET, *PBZDMA_TX_DESC_SET;

#ifdef _NEW_BZDMA_FROM_V8_
/* The structure of one segment of TX Descriptor for BLE only */
typedef union BZDMA_BLEONLY_TX_DESC_SEGMENT_ {
    struct {
        UINT32 start_addr:16;   /* bit[15:0], the start address of buffer SRAM */
        UINT32 len:11;          /* bit[26:16], the copied length*/
        UINT32 rsvd:2;          /* bit[28:27], reserved */
        UINT32 llid:2;          /* bit[30:29], the link layer id (only valid for data) */
        UINT32 isLast:1;        /* bit[31], is last segment (only valid for adv) */        
    };
    UINT32 DWord;
} BZDMA_BLEONLY_TX_DESC_SEGMENT, *PBZDMA_BLEONLY_TX_DESC_SEGMENT;
 
/* The structure of one fragment of TX Descriptor for BLE only */
typedef union BZDMA_BLEONLY_TX_DESC_FRAGMENT_ {
    struct {
        UINT32 start_addr:16;   /* bit[15:0], the start address of buffer SRAM */
        UINT32 len:11;          /* bit[26:16], the copied length*/
        UINT32 rsvd:4;          /* bit[30:27], reserved */
        UINT32 isLast:1;        /* bit[31], is last segment */
    };
    UINT32 DWord;
} BZDMA_BLEONLY_TX_DESC_FRAGMENT, *PBZDMA_BLEONLY_TX_DESC_FRAGMENT;

/* The structure of Tx Descriptor of BZDMA */
typedef struct BZDMA_BLEONLY_TX_DESC_SET_ { 
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
    BZDMA_BLEONLY_TX_DESC_SEGMENT  *seg; 
    BZDMA_BLEONLY_TX_DESC_FRAGMENT *frag;
#else
    BZDMA_BLEONLY_TX_DESC_SEGMENT  seg[BZDMA_LEONLY_TX_ENTRY_TOTAL_SEGS]; 
    BZDMA_BLEONLY_TX_DESC_FRAGMENT frag[BZDMA_LEONLY_TX_ENTRY_TOTAL_FRAGS];
#endif
} BZDMA_BLEONLY_TX_DESC_SET, *PBZDMA_BLEONLY_TX_DESC_SET;
#endif

/* The structure of one segment of RX Descriptor (only use for FW) */
typedef struct BZDMA_RX_DESC_SEGMENT_ {
    union {
        struct {
            UINT32 addr:16;   /* the address of destination address */
            UINT32 len:11;    /* the copied byte length */
            UINT32 flush:1;   /* flush data from rx fifo */
            UINT32 rsvd:4;    /* reserved */
        };
        UINT32 DWord;
    };
} BZDMA_RX_DESC_SEGMENT, *PBZDMA_RX_DESC_SEGMENT;

/*
            FW                                              HW  
 +-------------------------+     issue cmd to HW        +---------+
 |get a free entry to use  |--------------------------->|         |
 |                         |   complete status or INT   |  BZDMA  |
 |free memory & used entry |<---------------------------|         | 
 +-------------------------+                            +---------+
*/

/* The structure of TX descriptor entry status of BZDMA */
typedef struct BZDMA_TX_DESC_ENTRY_STATUS_ {
    BZDMA_TX_DESC_SEGMENT *pTxDesc; /* The pointer of multiple segments */
    UINT32 entry_idx:4;             /* my entry index*/
    UINT32 pkt_id:2;                /* current packet Id */
    UINT32 ch_id:2;                 /* current channel Id */
    UINT32 used:1;                  /* the entry is used ? */
    UINT32 piconet_id:2;            /* current piconet id */
    UINT32 total_len:11;            /* the total data length of all segments */
    UINT32 total_segs:4;            /* the total segements */  
    UINT32 is_sco_lt:1;             /* the logic link is sco ? */
#ifndef _NEW_HCI_DMA_DESIGN_FOR_ACL_SNIFF_SCHEDULE_
    UINT32 rsvd1:5;                 /* reserved */
#else
    UINT32 is_in_sniff:1;           /* the command is used in sniff ? */
    UINT32 rsvd1:4;                 /* reserved */    
#endif

    UINT32 le_mode_en:1;            /* Enable Txdma in LE mode */
    UINT32 le_pdu_cmd:2;            /* The Txdma PDU Command Type in LE */
    UINT32 le_conn_entry:6;         /* Connection Entry in LE */
    UINT32 lt_addr:3;               /* my lt_addr */
    UINT32 rsvd2:20;                /* reserved */
} BZDMA_TX_DESC_ENTRY_STATUS, *PBZDMA_TX_DESC_ENTRY_STATUS;

#ifdef _NEW_BZDMA_FROM_V8_
/* The structure of Ble Advertising TX descriptor entry status of BZDMA */
typedef struct BZDMA_BLEONLY_ADV_TX_DESC_ENTRY_STATUS_ {
    BZDMA_BLEONLY_TX_DESC_SEGMENT *pTxSegDesc;   /* The start pointer of multiple segments */
    BZDMA_BLEONLY_TX_DESC_FRAGMENT *pTxFragDesc; /* The start pointer of multiple fragments */    
    UINT32 max_segs:2;              /* maximum segments in the tx descriptor */ 
    UINT32 max_frags:4;             /* maximum fragments in one segment */  
    UINT32 le_pdu_cmd:2;            /* The Txdma PDU Command Type in LE */
    UINT32 rsvd1:24;                /* reserved */    
} BZDMA_BLEONLY_ADV_TX_DESC_ENTRY_STATUS, *PBZDMA_BLEONLY_ADV_TX_DESC_ENTRY_STATUS;

/* The structure of Ble Data TX descriptor entry status of BZDMA */
typedef struct BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS_ {
    BZDMA_BLEONLY_TX_DESC_SEGMENT *pTxSegDesc;   /* The pointer of multiple segments */
    BZDMA_BLEONLY_TX_DESC_FRAGMENT *pTxFragDesc; /* The pointer of multiple fragment */  
    UINT32 seg_wptr:4;              /* write pointer of segment table */                    
    UINT32 seg_rptr:4;              /* read pointer of segment table */
    UINT32 free_segs:5;             /* free packet counts of segment table */        
    UINT32 rsvd1:3;                 /* reserved */ 
    UINT32 bm_used_segs:16;         /* index bitmap of used segments */
} BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS, *PBZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS;

#endif

/* The structure of management of BZDMA */
typedef struct BZDMA_MANAGE_ {    
    BZDMA_TX_DESC_ENTRY_STATUS TxEntSta[BZDMA_TX_DESC_ENTRY_NUM];
    UINT16 bmFreeTxEnt;     /* free Tx command entry bitmap */

#ifdef _NEW_BZDMA_FROM_V8_
    BZDMA_BLEONLY_ADV_TX_DESC_ENTRY_STATUS BleAdvTxEntSta;
    BZDMA_BLEONLY_DATA_TX_DESC_ENTRY_STATUS BleTxEntSta[BZDMA_LEONLY_TX_MAX_ENTRY_NUM]; 
    UINT16 bmFreeBleTxEnt;  /* free Tx command entry bitmap */
#endif    
} BZDMA_MANAGE, *PBZDMA_MANAGE;

#ifdef _SCO_SEND_SINE_TONE_PKT_
extern SECTION_SRAM UINT16 Bzdma_Sw_SCO_Tx_Buf[3][48];
#endif

void bzdma_init(void);
UINT8 bzdma_get_free_tx_entry(void);
UINT8 bzdma_get_dedicated_free_tx_entry(UINT8 tx_type);
void bzdma_release_tx_entry(UINT8 entry_id);
UINT8 bzdma_get_free_rx_entry(void);
void bzdma_release_rx_entry(UINT8 entry_id);
void bzdma_send_txcmd(UINT8 entry_id);
void bzdma_send_rxcmd(UINT8 entry_id);
UINT8 bzdma_invalid_txcmd(UINT8 entry_id,UINT8 am_addr,UINT8 sco_id);
void bzdma_send_burst_rxcmd_and_wait_complete(BZDMA_RX_DESC_SEGMENT * pRxDesc,
         UINT8 cmd_count,UINT8 fifo_type,UINT8 sco_ch_id,UINT8 wait_complete);
void bzdma_memset_sco_tx_fifo(void);
SECTION_ISR_LOW void bzdma_int_handler(void);

void BB_dma_write_baseband_TX_FIFO(BZDMA_TX_DESC_SEGMENT * ptxdesc,
     UINT8 seg_num,UINT8 am_addr,UINT8 piconet_id,UINT8 sco_en,UINT8 sco_ch_id);
void BB_dma_write_baseband_SYNC_TX_FIFO(UCHAR *buf, 
                                                UINT16 length,UCHAR sco_ch_id);
void BB_dma_read_baseband_SYNC_RX_FIFO(UCHAR * buf, UINT16 length, 
                                       UCHAR sco_ch_id, UINT8 wait_complete);

#ifdef _NEW_BZDMA_FROM_V8_
UINT8 bzdma_get_ble_dedicated_free_tx_entry(UINT8 tx_type);
void bzdma_release_ble_tx_entry(UINT8 entry_id);
void bzdma_send_ble_txcmd(UINT8 entry_id);
UINT8 bzdma_invalid_ble_txcmd(UINT8 entry_id);
void bzdma_flush_ble_data_ring_fifo(UINT8 conn_entry, UINT8 flush_all);
UINT8 bzdma_send_packet_to_ble_data_ring_fifo(UINT8 conn_entry, UINT8 llid,
                            BZDMA_BLEONLY_TX_DESC_FRAGMENT *pfrag, UINT8 Nfrag);
UINT8 bzdma_update_fw_rptr_of_ble_data_ring_fifo(UINT8 conn_entry,
                           UINT8 free_cnts,UINT8 check_cam);

INLINE void bzdma_update_fw_wptr_of_ble_data_ring_fifo(UINT8 conn_entry, UINT8 write_pointer);
#endif

extern BZDMA_MANAGE Bzdma_Manager;
extern UINT8 bzdma_supported_le_link_num;
#ifdef _NEW_BZDMA_FROM_V8_PROGRAMMABLE_CONFIG_SETTING_
extern UINT8 bzdma_supported_le_max_seg_num;
extern UINT8 bzdma_supported_le_max_frag_num;
#endif
#endif

