/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       hci_transport_uart.h
* @brief     COM API - a generic Hardware independent Interface for Serial devices
*                               (rs232, USB, pipes, mailslots ...)
*
* @author   	gordon
* @date      	2015-07-13
* @version	v0.1
*/

#if !defined(__HCI_TRANSPORT_UART_H)
#define      __HCI_TRANSPORT_UART_H

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
    comeError
} TComEvent;

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


typedef TComResult ( * PComCallBack)(
                                           void     *pvContext,
                                           TComEvent event,
                                           uint32_t  uStatus,
                                           void     *pvBuffer,
                                           uint32_t  uLength
                                          );
COMENTRY comOpen (char        *pszDeviceName,
                  char        *pszParameter,
                  PComCallBack callBack,
                  void        *callBackContext,
                  HCOM        *phCom);


COMENTRY comClose (HCOM hCom);

COMENTRY comWrite (HCOM hCom, uint8_t *pvBuffer, uint32_t uBufferLength);

COMENTRY comResponse (HCOM hCom, uint8_t *pvBuffer);



#if defined (__cplusplus)
}
#endif

#endif

