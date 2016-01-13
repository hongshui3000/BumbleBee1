/***************************************************************/
/* $Header: /var/lib/cvs/sw/tool/flags.awk,v 1.23 2011/05/03 11:04:44 or Exp $ */
/*                                                             */
/* !!! This file is generated automatically          !!!       */
/* !!! Don't change it manually                      !!!       */
/* !!! For additional FLAGS use file 'ADDFLAGS.H'    !!!       */
/*                                                             */
/***************************************************************/

#ifndef __FLAGS_H
#define __FLAGS_H      1






#define UPPER_STACK_USE_NORMAL_HCI  1
#define UPPER_STACK_USE_VIRTUAL_HCI  0


#define OS_MAX_QUEUE_NAME           (5)
#define OS_MAX_POOL_NAME_ALIAS_FOR_EACH_POOL                   (1)         /*Max pool name alias for each pool*/
#define OS_HEAP_MEMORY_ALLOC_BYTES_COUNT 0x262c//0x23B0/**<0x2400*/ /**< gordon test 0x2400*/    /**<Heap use for stMemoryAlloc*/
#define OS_MODULE_WITH_QUEUENAMESET_COUNT 2         /**<Max. number of protocol module with more than one queuename*/

#define BT_RFCOMM_CREDIT_COUNT   4         /**<Credits for flow control*/
#define BT_RFCOMM_MTU_COUNT      330       /**<Max Packet Size (if 0 derived from BT_xx_MTU_L2C_BYTE_COUNT)*/
#define BT_SDP_SERVER_BUFFER_BYTE_COUNT 0         /**<size of Buffer used to build a response (on Stack)*/
#define BT_SDP_CLIENT_BUFFER_BYTE_COUNT 450       /**<size of buffer to assemble segmented response*/
#define BT_SDP_CLIENT_BUFFER_COUNT 1         /**<count for Buffer to assamble fragmeted response*/

/*----- offset used in some upstream msgs that allows upper layers ---*/
/*----- (e.g. BlueAPI) to re-use buffer (no copy ..). MUST be an   ---*/
/*----- even value (alignment, API packed structs conversion ..):  ---*/
#define BT_GATT_US_WRITE_OFFSET_COUNT (6 + 14 + 8)         /**<Offset used in upstream messages*/
#define BT_GATT_DS_WSIZE_COUNT   4         /**<Window size usable in notifications and write commands*/

#define BT_L2C_MAX_CMD_RETRIES_COUNT 0         /**<Max. Nbr of comand retries (if 0 reduced RAM and ROM size)*/
#define BT_L2C_MAX_RX_BUFFER_COUNT 0         /**<Streaming Max. nbr of RxBuffer send to upper layer*/
#define BT_HCI_FIXED_PACKET_TYPES_COUNT 0         /**<allowed packettypes the LLS may use, 0=HCI uses value depending on framesizes, see hci_code.h or HCI spec for values*/
#define BT_HCI_PAGE_TIMEOUT_DEFAULT_COUNT (0x2000)    /**<HCI default page timeout (0x2000 if undefined,5.12sec)*/
#define BT_HCI_INQIURY_RESULT_LOWWATER_COUNT 3         /**<Low Watermark for deviceInq results from system pool*/

#define BT_DS_BUFFER_COUNT       10        /**<Downstream buffer count*/

#define BT_DS_BUFFER_SIZE           (BT_DS_PDU_L2C_BYTE_COUNT + BT_DS_WRITE_OFFSET_COUNT)
#define BT_US_BUFFER_SIZE           (BT_US_PDU_L2C_BYTE_COUNT + BT_US_WRITE_OFFSET_COUNT + 5 + 4)//(BT_US_PDU_L2C_BYTE_COUNT + BT_US_WRITE_OFFSET_COUNT + ACL_HDR_LENGTH + L2CAP_HDR_LENGTH)
#define BT_RFCOMM_DS_BUFFER_COUNT 0         /**<Buffer count for RFCOMM (DS)*/
#define BT_L2CAP_DS_BUFFER_COUNT 1         /**<Buffer count for L2CAP (DS)*/
#define BT_US_BUFFER_COUNT       10        /**<Upstream buffer count*/
#define BT_DS_MTU_L2C_BYTE_COUNT 65535     /**<L2CAP DS MTU size*/
#define BT_US_MTU_L2C_BYTE_COUNT 65535     /**<L2CAP US MTU size*/
#define BT_DS_PDU_L2C_BYTE_COUNT 335       /**<L2CAP DS PDU size*/
#define BT_US_PDU_L2C_BYTE_COUNT 335       /**<L2CAP US PDU size*/
#define BT_US_WRITE_OFFSET_COUNT 24        /**<initial writeOffset for upstream data*/
#define BT_DS_WRITE_OFFSET_COUNT 24        /**<initial writeOffset for downstream data*/
#define BT_SYS_SHORT_BUFFER_BYTE_COUNT 72        /**<Short buffer size*/
#define BT_SYS_SHORT_BUFFER_COUNT 40 /**<gordon test 14 40*/        /**<Short buffer count*/
#define BT_SYS_MIDDLE_BUFFER_BYTE_COUNT 283       /**<Middle buffer size*/
#define BT_SYS_MIDDLE_BUFFER_COUNT 6 /**< gordon test 0 6*/        /**<Middle buffer count*/
#define BT_SYS_LONG_BUFFER_BYTE_COUNT 512       /**<Long buffer size*/
#define BT_SYS_LONG_BUFFER_COUNT 2 /**<gordon test 0 2*/         /**<Long buffer count*/
#define BT_SECMAN_POLICY_COUNT   6         /**<maximum number of policies*/
#define BT_LE_SECMAN_POLICY_COUNT 4

/*----define---*/
#define F_BT_GATT_SERVICE_CHANGED 1         /**<GATT service changed attribute supported*/
#define F_BT_L2C_ENHANCED_CONFORMANCE 0         /**<L2CAP enhanced conformation*/

#define F_BT_L2C_LE_DS_FRAGMENTATION 1         /**<Downstream Fragmentation supported*/

#define F_BT_L2C_EXT_FEATURE_SUPPORT 1/**<champion 0-->1*/          /**<Not used - L2CAP extended feature*/
#define F_BT_L2C_ENHANCED_FEATURE_SUPPORT 0     /**<champion 1-->0*/   /**<L2CAP enhanced feature support*/
#define F_BT_LE_BT41_SUPPORT 1
#define F_BT_BREDR_SUPPORT 1

#define F_BT_LOW_ENERGY          1         /**<support BT Low Energy*/
#define F_BT_LE_PRIVACY_MODE     0         /**<support BLE Privacy Mode (random resolvable address)*/
#define F_BT_LE_PRIVACY_RESOLVING 1         /**<support BLE Random Resolvable Address Resolution*/

#define F_BT_SCO                 1         /**<Synchronous Connection-Oriented  (experimentally)*/

#define SWITCH_ROLE_TRIES               2

 
#define VERSION_STRING			"Realtek_V0.1"



#endif
