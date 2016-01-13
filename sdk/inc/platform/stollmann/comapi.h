/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       comapi.h
* @brief     COM API - a generic Hardware independent Interface for Serial devices
*                               (rs232, USB, pipes, mailslots ...)
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#if !defined(__COMAPI_H)
#define      __COMAPI_H

#include <stdint.h>
#include <stdbool.h>

#if defined (__cplusplus)
extern "C" {
#endif

typedef  unsigned int HANDLE;




/**  TYPEDEFINITIONS, ENUMERATIONS and CONSTANT DEFINITIONS */
typedef HANDLE HCOM;                    /**< we use our own handle type */
#define COMENTRY  TComResult      /**< the return and call type   */


/** Events on a DTE Type Interface */
typedef enum tagComEvent
{
	/** general events */
	comeNone,
	comeOpenCompleted,
	comeCloseCompleted,
    comeDataIndication,
    comeDataTransmitted,
    comeError,
//    comeDebugOut,
//    comeResetCompleted,                           /**< status 0 = success, != 0 failure code */
//    comeTransmitData,
 //   comeGetBuffer,
//    comeDataIndicationBuf,

    /** serial device control lines and status events */
//    comeBreak = 0x10,
//    comeDSR,
//    comeCTS,
//    comeDCD,
//    comeRING,

    /** connection events */
 //   comeConnected = 0x40,
//    comeCallFailed,
//    comeDisconnected,
//    comeHangupTerminated,
//    comeIncomingCall,

    /** PnP Events */
 //   comeAdapterPlugin = 0x50,
 //   comeAdapterPlugout,
//    comeAdapterSuspend,
//    comeAdapterResumed,
  
 //   comeStartTimer = 0x60,
 //   comeStopTimer,
  
 //   comeLastEvent = comeStopTimer
} TComEvent;
#if 0
typedef enum tagHandShakeMode
{
	hsmHandShakeHardware,
	hsmHandShakeSoftware,
	hsmHandShakeOff,
	hsmHandShakeNoCTSRTS,
	hsmHandShakeMode4
} THandShakeMode;
#endif
#if 0
/** Commands on a DTE Type Interface  */
typedef enum tagComProperty
{
	/** general functions */                       /**< parameters: */
	compHardwareProvider,                         /**< uint16_t IFtype */
	compFriendlyName,
	compHCIChannelMode,                           /**< uint16_t 0==Single,Other==MultiChannel       */
	compResetController,                          /**< uint16_t reset type */
                                                /**< if Reset is not supported by the HW/
                                                   controller, comResultPropertyNotImplemented
                                                   or comResultPropertyIgnored can be returned */

	/** serial device control lines and data format */
	compBaudRate = 0x10,                          /**< uint32_t baudrate                        */
	compDTR,                                      /**< bool: ON or OFF                          */
	compRTS,                                      /**< bool: ON or OFF                          */
	compByteSize,                                 /**< uint32_t bits per Byte                   */
	compParity,                                   /**< uint32_t ??????                          */
	compStopBits,                                 /**< uint32_t number of stopbits              */
	compMaximumBaudrate,                          /**< uint32_t max baudrate                    */
    compResetTimeout,                             /**< uint32_t reset timeout                   */
    compInitialBaudrate,                          /**< uint32_t baudrate after reset            */
    compResetComplete,                            /**< uint32_t timeout for reset complete      */
    compResetAnswer,                              /**< uint32_t timeout for answer to reset cmd */
    compHandShakeMode,                            /**< uint32_t 0 = hw, 1 = sw, 2 = off         */
    compCTS,
    compRLSD,
    compDSR,

    /** following commands exist only for DCE interfaces (pty) */
    compDCD           = 0x20,                     /**< bool: ON or OFF                          */
    compRING,                                     /**< bool: ON or OFF                          */
  
    /** Channel Configuration */
    compBChannelStack = 0x30,
    /** is the channel ready for data transmision ? */
    compChannelReady,                             /**< bool: TRUE is ready */
 
    /** Connection control interface */
    compConnect = 0x40,
    compListen,
    compDisconnect,
    compCallAccept,
    compCallReject,

    compBroadcastRequestType = 0x50,             /**< see bfbc.h  TbcEvent    */

    compDeviceHandle         = 0x60,             /**< Raw device handle from OS */
    compPollForRxData        = 0x61,

    compTransportPin         = 0x0201,           /**< blueControl+ obex       */
    compTransportEncryption  = 0x0202,           /**< properties pass trough  */
    compTransportLinkHandle  = 0x0203,
    compGetInterface         = 0x0300,
    compGetInterfaceEx       = 0x0301,
    compDebugFlags           = 0x0302,
    compTimerPoll            = 0x0303,
    compTimerSet             = 0x0304,
  
    compFlushTxQueue,

    comLastProperty          = compFlushTxQueue
  
} TComProperty;


/** Property type specific definitions  */
/** GetProperty for compHardwareProvider - Parameter Size: uint16_t */
#define BT_USB_HARDWARE       0x01
#define BT_SERIAL_HARDWARE    0x02
#define BT_ATMEL_PCMCIA       0x03
#define BT_WAVEPLUS_PCMCIA    0x04
#define BT_SERIAL_HARDWARE_A  0x05   /**< serial HCI H4 with guaranteed frame boundaries */

/** PutProperty for compStopBits - Parameter Size: uin16_t */
#define COM_ONESTOPBIT   1
#define COM_TWOSTOPBITS  2

/** PutProperty for compParity - Parameter Size: uint16_t */
#define COM_NOPARITY    1
#define COM_ODDPARITY   2
#define COM_EVENPARITY  3
#endif

typedef enum tagComResult
{
	comResultSuccess     = 0,             /*  0: great, lets go on  */
	comResultPending     = 1,             /*  1: request is pending */

    /** 0 - 0x0F reserved for positiv results */
    comResultUnspecified = 0x10,            /**< 10: unspecified error */
    comResultNoResources,                   /**< 11: no Memory etc */
    comResultOpenfailed,                    /**< 12: open of SubDevice failed */
    comResultHwProviderNotImplemented,      /**< 13: the requested Hardware Provider is not implemented */
    comResultTransferModeNotImplemented,    /**< 14: the requested Mode is not implemented */
    comResultInvalidHandle,                 /**< 15: the handle is invalid */
    comResultInvalidLength,                 /**< 16: the length is invalid */
    comResultInvalidState,                  /**< 17: the given channel state is invalid */
    comResultInvalidParam,                  /**< 18: the param are invalid */

    comResultPropertyNotImplemented = 0x30, /**< 30: the requested Property is not implemented */
    comResultPropertyIgnored,               /**< 31: the requested Property is ignored */
    comResultPropertyBadPara,               /**< 32: Property parameter is bad */
    comResultPropertySetFailed,             /**< 33: Setting of Property parameter failed */
    comResultNotReady,                      /**< The device is not ready RNR" */
    comResultTimeout,                       /**< Sync operation failed */

    comResultLast = comResultTimeout
} TComResult;


/**  MAKROS and additional DEFINES  */
//#define COM_SUCCESS(result)  (result < comResultUnspecified)

/** INTERFACE FUNCTIONS   */
/** Client Data Callback - called by COMAPI Provider  */
/****************************************************************************/
/** Function name  : ComCallBack                                             */
/** Description    :                                                         */
/** Return type    : none                                                    */
/** Argument         : pvContext   the context given in comOpen()            */
/** Argument         : event       the event type                            */
/** Argument         : uStatus     a status word                             */
/** Argument         : pvBuffer    pointer to type dependent argument buffer */
/** Argument         : uLength     number of the bytes in the argument buffer*/
typedef TComResult ( * PComCallBack)(
                                           void     *pvContext,
                                           TComEvent event,
                                           uint32_t  uStatus,
                                           void     *pvBuffer,
                                           uint32_t  uLength
                                          );

/** Open - called by COMAPI Client                                           */
/****************************************************************************/
/** Function name  : comOpen                                                 */
/** Description    : opens a communication channel                           */
/** Return type    : TComResult       see definition of enum                 */
/** Argument       : pszDeviceName    the name of device to open             */
/** Argument       : pszPortName      specific channel within the device     */
/** Argument       : callBack         pointer to client callback function    */
/** Argument       : callBackContext  a client specific context pointer,     */
/** will be used as argument for the callback                                */
/** Argument       : phCom            pointer to a var of type HCOM used as id*/
/** in all function calls                                                    */
COMENTRY comOpen (char        *pszDeviceName,
                  char        *pszParameter,
                  PComCallBack callBack,
                  void        *callBackContext,
                  HCOM        *phCom);
#if 0
#define COM_CALLBACK_WINDOW    0x0001  /**< callBackParam is hWnd, callBackContext is WM_OFFSET */
                                       /**< window message is WM_OFFSET + TComEvent */
#define COM_CALLBACK_FUNCTION  0x0002  /**< callBackParam is PComCallBack */
#define COM_CALLBACK_THREAD    0x0004  /**< callBackParam is thread handle, callBackContext is WM_OFFSET */
                                       /**< message id is WM_OFFSET + TComEvent */

COMENTRY comOpenEx (char     *pszDeviceName,
                    char     *pszParameter,
                    uint32_t  uCallBackType,
                    void     *callBackParam,
                    void     *callBackContext,
                    HCOM     *phCom);
#endif

/** Close - called by COMAPI Client                                          */
/****************************************************************************/
/** Function name  : comClose                                                */
/** Description    : close the communication channel identified by hCom      */
/** Return type    : TComResult              see definition of enum          */
/** Argument       : hCom                    the handle provided by comOpen()*/
COMENTRY comClose (HCOM hCom);

/** Reset - called by COMAPI Client                                          */
/****************************************************************************/
/** Function name  : comReset                                                */
/** Description    : reset the communication channel identified by hCom      */
/** Return type    : TComResult              see definition of enum          */
/** Argument       : hCom                    the handle provided by comOpen()*/
//COMENTRY comReset (HCOM hCom);

/** Write - called by COMAPI Client                                          */
/****************************************************************************/
/** Function name : comWrite                                                 */
/** Description   : writes uBufferLength chars to the communication channel  */
/** Return type   : TComResult              see definition of enum           */
/** Argument      : hCom                    the handle provided by comOpen() */
/** Argument      : pvBuffer                pointer to a buffer containing the data*/
/** Argument      : uBufferLength           number of byte to write          */
COMENTRY comWrite (HCOM hCom, uint8_t *pvBuffer, uint32_t uBufferLength);

/** confirma last indicated RX buffer                                        */
/****************************************************************************/
/** Function name : comResponse                                              */
/** Description   : Indicate consumptuion of last signaled RX buffer         */
/** Return type   : TComResul               see definition of enum           */
/** Argument      : hCom                    the handle provided by comOpen() */
/** Argument      : pvBuffer                pointer to a buffer containing the data*/
COMENTRY comResponse (HCOM hCom, uint8_t *pvBuffer);

/** SetProperty - called by COMAPI Client                                    */
/****************************************************************************/
/** Function name  : comSetProperty                                          */
/** Description    : set device properties                                   */
/** Return type    : TComResult    see definition of enum                    */
/** Argument       : hCom          the handle provided by comOpen()          */
/** Argument       : comProperty   type of property to change                */
/** Argument       : pvData        pointer to type dependent argument buffer */
/** Argument       : uDataLength   number of the bytes in the argument buffer*/
//COMENTRY comSetProperty (HCOM hCom, TComProperty comProperty, void *pvData, uint32_t uDataLength);

/** GetProperty - called by COMAPI Client                                    */
/****************************************************************************/
/** Function name  : comGetProperty                                          */
/** Description    : request Property info from device                       */
/** Return type    : TComResult       see definition of enum                 */
/** Argument       : hCom          the handle provided by comOpen()          */
/** Argument       : comProperty   type of property to change                */
/** Argument       : pvData        pointer to type dependet argument buffer  */
/** Argument       : puDataLength  pointer to UINT var IN: max length of the argument buffer*/
/** OUT: used length in arg buffer                                           */
//COMENTRY comGetProperty (HCOM hCom, TComProperty comProperty, void *pvData, uint32_t *puDataLength);

/** Connection Control */
//COMENTRY comCallControl (HCOM hCom, TComProperty comProperty, void *pvParam);

/** Register PnP Handler */
//COMENTRY comRegisterPnpHandler (PComCallBack callBack, void *callBackContext);

/** Enumeration Support Callback function - called by COMAPI Provider        */
/****************************************************************************/
/** Function name  : ComEnumCallBack                                         */
/** Description    : reports a driver name found during enumeration process  */
/** Return type    : none                                                    */
/** Argument       : pvContext              the context given in comEnumAdaters()*/
/** Argument       : pszDriverName          the driver name                  */
/** Argument       : pszFriendlyName        a user readable name             */
#if 0
typedef void ( * PComEnumCallBack)(
                                         void *pvContext,
                                         char *pszDriverName,
                                         char *pszFriendlyName
                                        );

/** Enumeration Support EnumAdapters - called by COMAPI Client               */
/****************************************************************************/
/** Function name  : comEnumAdapters                                         */
/** Description    : Enum all active COM adapters for this API. Get back a driver name in*/
/** callBack procedure for later use in 'comOpen'                            */
/** Return type:     number of adapters found                                */
/** Argument       : callBack      pointer to the client callback function   */
/** Argument       : pvContext     a client specific context pointer,        */
/** will be used as argument for the callback                                */
uint32_t  comEnumAdapters (PComEnumCallBack callBack, void *pvContext);

/** Function pointer declarations of the Interface functions                 */
/** may be used in the Provider                                              */
typedef uint32_t ( * PComEnumAdapters)(
                                             PComEnumCallBack callBack,
                                             void            *pvContext
                                            );

typedef TComResult ( * PComOpen)(
                                       char        *pszDeviceName,
                                       char        *pszPortName,
                                       PComCallBack callBack,
                                       void        *callBackContext,
                                       HCOM        *phCom
                                      );

typedef TComResult ( * PComClose)(HCOM hCom);

typedef TComResult ( * PComWrite)(
                                        HCOM      hCom,
                                        uint8_t  *puBuffer,
                                        uint32_t  uBufferLength
                                       );

typedef TComResult ( * PComResponse)(
                                           HCOM     hCom,
                                           uint8_t *puBuffer
                                          );

typedef TComResult ( * PComSetProperty)(
                                              HCOM         hCom,
                                              TComProperty comProperty,
                                              void        *pvData,
                                              uint32_t     uDataLength
                                             );

typedef TComResult ( * PComGetProperty)(
                                              HCOM         hCom,
                                              TComProperty comProperty,
                                              void        *pvData,
                                              uint32_t    *puDataLength
                                              );

typedef TComResult ( * PComCallControl)(
                                              HCOM         hCom,
                                              TComProperty comProperty,
                                              void        *pvParam
                                             );

typedef TComResult ( * PComRegisterPnpHandler)(
                                                     PComCallBack callBack,
                                                     void        *callBackContext
                                                    );

typedef TComResult ( * PComReset)(HCOM hCom);

/** Synchronous low level access for special register based work mode */
typedef TComResult ( * PComWriteByte)(HCOM hCom, uint8_t data);
typedef TComResult ( * PComReadByte)(HCOM hCom, uint8_t *pData);

typedef struct tagCOMAPIInterface
{
	uint32_t               uLength;
	PComEnumAdapters       comEnumAdapters;
	PComOpen               comOpen;
	PComClose              comClose;
	PComWrite              comWrite;
	PComResponse           comResponse;
	PComSetProperty        comSetProperty;
	PComGetProperty        comGetProperty;
    PComCallControl        comCallControl;
    PComRegisterPnpHandler comRegisterPnpHandler;
    PComReset              comReset;
} TComApiInterface, *PComApiInterface;

#define defTComApiInterface                                 \
  0                          /**< uint32_t               uLength               */ \
  , (PComEnumAdapters)0      /**< PComEnumAdapters       comEnumAdapters       */ \
  , (PComOpen)0              /**< PComOpen               comOpen               */ \
  , (PComClose)0             /**< PPComClose             comClose              */ \
  , (PComWrite)0             /**< PComWrite              comWrite              */ \
  , (PComResponse)0          /**< PComResponse           comResponse           */ \
  , (PComSetProperty)0       /**< PComSetProperty        comSetProperty        */ \
  , (PComGetProperty)0       /**< PComGetProperty        comGetProperty        */ \
  , (PComCallControl)0       /**< PComCallControl        comCallControl        */ \
  , (PComRegisterPnpHandler)0/**< PComRegisterPnpHandler comRegisterPnpHandler */ \
  , (PComReset)0             /**< PComReset              comReset              */

typedef struct tagCOMAPIInterfaceEx
{
    uint32_t               uLength;
	PComEnumAdapters       comEnumAdapters;
	PComOpen               comOpen;
    PComClose              comClose;
    PComWrite              comWrite;
    PComResponse           comResponse;
    PComSetProperty        comSetProperty;
    PComGetProperty        comGetProperty;
    PComCallControl        comCallControl;
    PComRegisterPnpHandler comRegisterPnpHandler;
    PComReset              comReset;

    PComWriteByte          comWriteByte;
    PComReadByte           comReadByte;
} TComApiInterfaceEx, *PComApiInterfaceEx;

COMENTRY comGetInterface (PComApiInterface comIF);
COMENTRY comGetInterfaceEx (PComApiInterfaceEx comIF);


typedef TComResult ( * PComGetInterface)(PComApiInterface comIF);
#endif

#if defined (__cplusplus)
}
#endif

#endif /**< if !defined(__COMAPI_H) */
