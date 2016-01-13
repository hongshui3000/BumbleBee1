
#if !defined(__MESSAGE_H)
#define      __MESSAGE_H
#define __MESSAGE__


#include <rtl_types.h>
#include <flags.h>


/** Management Entity type Identifier */

#define PH_TYPE                         0x0100        /**< Physical Link Contrl */

#define HCI_TYPE                        0x7100
#define SEC_TYPE                        0x7600

#define BT_TYPE                         0x8000        /**< Bluetooth generic */
#define SECMAN_TYPE                     0x8800        /**< Security Manager 2.1 */
#define BLE_TYPE                        0x8900        /**< Bluetooth Low Energy */
#define GATT_TYPE                       0x8A00        /**< Bluetooth Generic Attribute Prot. */


#define REQUEST                         0x0000
#define CONFIRMATION                    0x0040
#define INDICATION                      0x0080
#define RESPONSE                        0x00c0

/** Basic Message Type */
#define BM_DATA                            0x0003  /**< Normal data flow */
#define BM_REGISTER                        0x0011
#define BM_CONFIGURATION                   0x0013
#define BM_AUTHENTICATION                  0x0015
#define BM_HDP                             0x0038 /**< HDP Profile */
#define BM_DEVICE                          0x003A /**< device recognition (NFC) */
#define BM_MESSAGE                         0x003D
#define BM_LE_MESSAGE                      BM_MESSAGE


/** BT_HDP messages */
#define BLUE_API_MSG                    (BT_TYPE|BM_HDP|INDICATION)

/** Layer 1 Primitives */
#define PH_DATA_REQ                     (PH_TYPE|BM_DATA|REQUEST)
#define PH_DATA_IND                     (PH_TYPE|BM_DATA|INDICATION)

/** Message structure definitions  */

/**  xx_DATA_yyy */

/** DATA_CB_T.Flag and DATA_CB_EXT_T.Flag definitions */
#define DATA_CB_RELEASE        0x0001  /**< release buffer after sending       */
#define LONG_HDLC_FRAME        0x0002  /**< long data (L1 -> L2)               */
#define STORE_TO_NVRAM         0x0004  /**< directly store DATA to NVRAM */
#define DATA_CB_START_AUTOBAUD 0x0008
#define DATA_CB_STOP_AUTOBAUD  0x0010
#define DATA_CB_AUTOBAUD_ECHO  0x0020
#define DATA_CB_MORE           0x0040  /**< indicate that there is another data
                                          block available (for X.25)*/

#define DATA_CB_EXT_SET_PAR    0x0080  /**< SET_PARAMETER_REQ/PARAMETER_T */
                                       /**< data included                 */

#define DATA_CB_BLOCK_FIRST    0x0100  /**< First data block                   */
#define DATA_CB_BLOCK_MIDDLE   0x0200  /**< Continuation data block            */
#define DATA_CB_BLOCK_LAST     0x0400  /**< Last data block                    */

#define DATA_CB_NON_RELIABLE_DATA  0x0800  /**< non Reliable data              */

#define DATA_CB_EXPEDITED      0x1000  /**< expedited data                     */

typedef struct
{
	uint16_t      Flag;
	uint16_t      Offset;               /**< offset to data in data buffer */
	uint16_t      Length;               /**< length of data */
#if UPPER_STACK_USE_VIRTUAL_HCI
    uint8_t      Pkt_type;
#endif
    uint8_t   *BufferAddress;       /**< buffer address */
} DATA_CB_T;
typedef DATA_CB_T  * DATA_CB_P;
typedef CONST DATA_CB_T  * C_DATA_CB_P;

typedef struct
{
	DATA_CB_T  DataCB;
	THandle    Handle;
} DATA_CB_EXT_T;

typedef struct
{
	/** first part MUST be same as DATA_CB_T: !!!! */
	uint16_t      Flag;
	uint16_t      Offset;               /**< offset to data in data buffer */
	uint16_t      Length;               /**< length of data */
#if UPPER_STACK_USE_VIRTUAL_HCI
    uint8_t      Pkt_type;
#endif
    uint8_t   *BufferAddress;       /**< buffer address */
	/** extension   */
	uint16_t      Channel;              /**< Channel Identifier */
	uint16_t      TotalLength;          /**< total length of data (more-bit) */
} DATA_CB_CHAN_T;
typedef DATA_CB_CHAN_T  * DATA_CB_CHAN_P;

typedef struct
{
    uint8_t      QueueID;              /**< Source queue ID */
    uint8_t      TimerID;              /**< module timer ID */
    uint16_t      TimerChannel;         /**< module timer channel */
    uint32_t     IntervalInMS;         /**< timer interval in milliseconds */
} TIMER_T;



#define M_DATA_LENGTH       0x10        /**< length of message = header (6 bytes + 2 fill bytes) + M_DATA_LENGTH = 24 */

/** BlueFace interface */
typedef struct _TBlueFaceMessageReq
{
	PVOID    ApplHandle;     /**< Application Handle */
	DATA_CB_T DataCB;         /**< normal DataCB */
} TBlueFaceMessageReq;
typedef TBlueFaceMessageReq  * PBlueFaceMessageReq;

/** all messages to use BlueFace services have this format */
typedef union _TBlueFaceMData
{
	TBlueFaceMessageReq    MessageReq;
}
TBlueFaceMData;
typedef TBlueFaceMData  * PBlueFaceMData;


/** get additional type for the message parameter overly                     */
/** from external headers                                                    */
/** that helps to keep this file smal and clearly arranged                   */

#include <bt_msg.h>                /**< get the Bluetooth specific stuff */

/** message parameter overlay.                                               */
/** NOTE: when adding new parameters assure that the size of the union is    */
/**       NOT increased !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    */
typedef union M_DATA_T
{
	uint8_t                        MessageData[M_DATA_LENGTH];
	DATA_CB_T                   DataCB;
	DATA_CB_EXT_T               DataCBExt;
	DATA_CB_CHAN_T              DataCBChan;

	TIMER_T                     Timer;

	TBlueFaceMData              BlueFace;
#if !defined(BOOT)
	BT_MDATA_MEMBERS
#endif /**< (F_BLUE_TOOTH) */
} M_DATA_T;
typedef M_DATA_T *M_DATA_P;

typedef struct _MESSAGE_T
{
    uint16_t           Command;
    M_DATA_T       MData;                 /**< sizeof(M_DATA_T) = M_DATA_LENGTH */
} MESSAGE_T;
typedef MESSAGE_T *MESSAGE_P;

#endif /**< __MESSAGE_H */
