/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       bt_api.h
* @brief     BT application interface
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#ifdef __cplusplus
extern "C" {
#endif
#include <flags.h>
#include <btman.h>
blueFaceStatus btApiBT_AUTH_REQ(BLINKHANDLE bLinkHandle);

blueFaceStatus btApiBT_CON_REQ         (BAPPHANDLE appHandle, BLINKCONTEXT context,
                                        uint8_t * bd,            uint16_t frameSize,
                                        uint16_t psm,             uint16_t uuid,
                                        uint16_t lenPara,         uint8_t * para);

blueFaceStatus btApiBT_CON_RESP(BLINKHANDLE bLinkHandle, BLINKCONTEXT bLinkContext,
    uint8_t accept, uint16_t lenPara, uint8_t * para);

blueFaceStatus btApiBT_DISC_REQ(BLINKHANDLE bLinkHandle, uint16_t cause, uint16_t holdLink);
blueFaceStatus btApiBT_DISC_RESP(BLINKHANDLE bLinkHandle);

blueFaceStatus btApiBT_HCI_CLASS_REQ   (BAPPHANDLE appHandle, uint32_t serviceClass);

blueFaceStatus btApiBT_HCI_INQUIRY_REQ (BAPPHANDLE appHandle, uint16_t TimeOut,
                                        uint16_t       MaxBd,
                                        uint8_t idxIAC);

/** classMask and classValue can be used to filter for specific class of devices. use 0,0
   values for neutral (see all devices) inquiries */
blueFaceStatus btApiBT_HCI_KEY_RESP(uint16_t keyRequestType, uint16_t cause,
    uint16_t keyLength, uint8_t *pKey, uint8_t *bd);

blueFaceStatus btApiBT_HCI_NAME_REQ    (BAPPHANDLE appHandle, uint8_t * Bd);

blueFaceStatus btApiBT_PAGEMODE_REQ    (uint8_t pageMode);

blueFaceStatus btApiBT_SDP_ATTRIBUTE_REQ(BLINKHANDLE bLinkHandle, uint16_t maxLen,
    uint32_t serviceHandle, uint16_t lenAttrib, uint8_t *pAttrib);

blueFaceStatus btApiBT_SDP_SEARCH_REQ(BLINKHANDLE bLinkHandle, uint16_t maxHandles,
    uint16_t lenUuid, uint8_t *pUuid);

blueFaceStatus btApiDEVICE_DATA_GET_RESP(PVOID handle, uint16_t status, PDEVICE_DATA pDeviceData);

blueFaceStatus btApiDEVICE_DATA_SET_RESP(PVOID handle, uint16_t status, PDEVICE_DATA pDeviceData);

blueFaceStatus btApiGATT_ATTRIB_UPDATE_REQ(PVOID reqHandle, PVOID serviceHandle,
    uint16_t attribIndex, uint16_t length, uint16_t offset, uint8_t *pBuffer);

blueFaceStatus btApiGATT_ATTRIB_READ_RESP(BLINKHANDLE bLinkHandle, PVOID serviceHandle,
    uint16_t cause, uint16_t length, uint16_t offset, uint8_t *pBuffer);

blueFaceStatus btApiGATT_ATTRIB_WRITE_RESP(BLINKHANDLE bLinkHandle, uint16_t cause,
    uint16_t length, uint16_t offset, uint8_t *pBuffer);


blueFaceStatus btApiGATT_EXECUTE_WRITE_RESP(BLINKHANDLE bLinkHandle,
    uint16_t cause, uint16_t handle);

blueFaceStatus btApiGATT_EXECUTE_WRITE_REQ(BLINKHANDLE bLinkHandle, uint8_t flags);

blueFaceStatus btApiGATT_DISCOVERY_REQ(BLINKHANDLE bLinkHandle,
    uint16_t type, uint16_t startingHandle, uint16_t endingHandle, uint16_t uuid16, uint8_t *pUuid128);

blueFaceStatus btApiGATT_DISCOVERY_RESP(BLINKHANDLE bLinkHandle, uint16_t type,
    uint16_t startingHandle, uint16_t endingHandle);

blueFaceStatus btApiGATT_ATTRIB_READ_REQ(BLINKHANDLE bLinkHandle,
    uint16_t valueOffset, uint16_t nbrOfHandles, LPWORD pHandles);

blueFaceStatus btApiGATT_ATTRIB_READ_REQ_UUID(BLINKHANDLE bLinkHandle,
    uint16_t startingHandle, uint16_t endingHandle, uint16_t uuid16, uint8_t *pUuid128);

blueFaceStatus btApiGATT_ATTRIB_WRITE_REQ(BLINKHANDLE bLinkHandle, uint16_t type,
    uint16_t handle, uint16_t length, uint16_t writeOffset, uint16_t offset, uint8_t *pBuffer);

blueFaceStatus btApiGATT_LE_SCAN_REQ(uint8_t enable, uint8_t scanType, uint16_t scanInterval,
    uint16_t scanWindow, uint8_t filterPolicy, uint8_t localBdType, uint8_t filterDuplicates);

blueFaceStatus btApiGATT_LE_CONNECTION_UPDATE_REQ(BLINKHANDLE bLinkHandle,
    uint16_t connIntervalMin, uint16_t connIntervalMax, uint16_t connLatency,
    uint16_t supervisionTimeout, uint16_t minimumCELength, uint16_t maximumCELength);

blueFaceStatus btApiGATT_SET_RANDOM_ADDRESS_REQ(uint8_t * bd);

void btApiBT_HandleConReq(LPbtApplication app, LPblueFaceMsg pmsg);

#ifdef __cplusplus
}
#endif
