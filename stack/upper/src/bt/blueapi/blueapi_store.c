/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       blueapi_store.c
* @brief     BlueAPI Device Data Store (non volatile Pairing & CCC Bit storage)
* @details   
*
* @author   	gordon
* @date      	2015-07-08
* @version	v0.1
*/

#include <flags.h>
#include <blueface.h>
#include <btglbdef.h>
#include <bt_api.h>
#include <crc16btx.h>
#include <blueapi_types.h>
#include <os_pool.h>
#include <blueapi_def.h>
#include <stextif.h>

/* (F_GATTDEMO) */
#define TRACE_MODULE_ID  MID_BT_HDP
#define PEER_IDX_UNUSED     0xFF

typedef enum _TBlueAPI_DSComp
{
	dataStoreComp_NoMatch         = 0x00,
	dataStoreComp_Match           = 0x01,
	dataStoreComp_NoMatch_Changed = 0x02,
	dataStoreComp_Match_Changed   = 0x03,
} TBlueAPI_DSComp;

typedef TBlueAPI_DSComp (* DataStoreEntryCompareFunc)(PBlueAPI_DSEntry pEntry, PVOID pCompareFuncData);

/**
* @brief  get nv data size
* 
* @param pBlueAPIdata
*
* @return  nvSize
*
*/
uint16_t blueAPIStore_GetNVDataSize(PBlueAPI_Data pBlueAPIdata)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t        nvSize      = offsetof(TBlueAPI_DSNVData, dataDyn);

	nvSize += sizeof(TBlueAPI_DSPeer)   * pBlueAPI_DS->peerCount;
	nvSize += sizeof(TBlueAPI_DSEntryS) * pBlueAPI_DS->entriesSCount;
	nvSize += sizeof(TBlueAPI_DSEntryL) * pBlueAPI_DS->entriesLCount;

	return nvSize;
} /* blueAPIStore_GetNVDataSize */

/**
* @brief  blueapi save
*
* @param  pBlueAPIdata:
*
* @return  
*
*/
BOOL blueAPIStore_Save(PBlueAPI_Data pBlueAPIdata)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;

	if (!pBlueAPI_DS->changed)	/*no change no save*/
	{
		return FALSE;
	}

	pBlueAPI_DS->changed = FALSE;

	if (pBlueAPIdata->ConfigParameter.StoreBondMode == blueAPI_StoreBondModeNVStore)
	{
		pBlueAPI_DS->pNVData->maxPeerCount  = pBlueAPI_DS->maxPeerCount;
		pBlueAPI_DS->pNVData->firstPeerIdx  = pBlueAPI_DS->firstPeerIdx;
		pBlueAPI_DS->pNVData->lastPeerIdx   = pBlueAPI_DS->lastPeerIdx;
		pBlueAPI_DS->pNVData->size          = blueAPIStore_GetNVDataSize(pBlueAPIdata);

		pBlueAPI_DS->pNVData->checksum = btxfcs(BTXFCS_INIT,
		                                        ((uint8_t *)pBlueAPI_DS->pNVData) + offsetof(TBlueAPI_DSNVData, size),
		                                        pBlueAPI_DS->pNVData->size - offsetof(TBlueAPI_DSNVData, size)
		                                        );

		BLUEAPI_TRACE_PRINTF_2(BLUEAPI_TRACE_MASK_TRACE,
		                           "blueAPIStore_Save: nvSize:0x%x chksum:0x%x",
		                           pBlueAPI_DS->pNVData->size,
		                           pBlueAPI_DS->pNVData->checksum
		                           );

		stNvramWrite(pBlueAPI_DS->pNVData, pBlueAPI_DS->pNVData->size);
	}

	return TRUE;
} /* blueAPIStore_Save */

/**
* @brief  load from nvdata and verify checksum
*
* @param  pBlueAPIdata:
*
* @return  load result(with checksum)
*
*/
BOOL blueAPIStore_Load(PBlueAPI_Data pBlueAPIdata)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t        nvSize      = blueAPIStore_GetNVDataSize(pBlueAPIdata);
	uint16_t        checksum;

	stNvramInit(nvSize);
	stNvramRead(pBlueAPI_DS->pNVData, nvSize);

	pBlueAPI_DS->maxPeerCount = pBlueAPI_DS->pNVData->maxPeerCount;
	pBlueAPI_DS->firstPeerIdx = pBlueAPI_DS->pNVData->firstPeerIdx;
	pBlueAPI_DS->lastPeerIdx  = pBlueAPI_DS->pNVData->lastPeerIdx;

	checksum = btxfcs(BTXFCS_INIT,
	               ((uint8_t *)pBlueAPI_DS->pNVData) + offsetof(TBlueAPI_DSNVData, size),
	               nvSize - offsetof(TBlueAPI_DSNVData, size)
	               );

	BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
	                         "blueAPIStore_Load: nvSize:0x%x/0x%x chksum:0x%x/0x%x",
	                         pBlueAPI_DS->pNVData->size, nvSize,
	                         pBlueAPI_DS->pNVData->checksum, checksum
	                         );

	return (pBlueAPI_DS->pNVData->size == nvSize) &&
	     (pBlueAPI_DS->pNVData->checksum == checksum);
} /* blueAPIStore_Load */

/**
* @brief		find peer device or allocate
*
* @param	pBlueAPI_DS:
* @param	bd:  
* @param	bdType: 
* @param	allocate: 
*
* @return
*
*/
uint16_t blueAPIStore_FindAllocatePeer(PBlueAPI_DS pBlueAPI_DS, TBdAddr bd, uint8_t bdType, BOOL allocate)
{
	uint16_t peerIdx  = PEER_IDX_UNUSED;
	uint16_t useCount = 0;
	uint16_t i;

	for (i = 0; i < pBlueAPI_DS->peerCount; i++)
	{
		PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[i];

		if (pPeer->used == 0x01)
		{
			useCount++;
			/* ignore RESOLVED bit for bdtype (resolved and public share same bond) */
			if (((pPeer->bdType & ~(BLUEFACE_BDTYPE_LE_RESOLVED_MASK)) ==
			   (bdType & ~(BLUEFACE_BDTYPE_LE_RESOLVED_MASK))
			  ) &&
			  (memcmp(pPeer->bd, bd, BD_ADDR_SIZE) == 0)
			 )
			{
				return i;
			}
		}
		else if (allocate && (peerIdx == PEER_IDX_UNUSED))
		{
			/* first unused peer entry */
			peerIdx = i;
		}
	}

	/* no more peers allowed */
	if (allocate && (useCount >= pBlueAPI_DS->maxPeerCount))
	{
		return PEER_IDX_UNUSED;
	}

	if (peerIdx != PEER_IDX_UNUSED)
	{
		PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[peerIdx];

		memset(pPeer, 0x00, sizeof(TBlueAPI_DSPeer));

		/* allocate peer */
		pPeer->used         = 0x01;
		pPeer->nextPeerIdx  = PEER_IDX_UNUSED;
		pPeer->bdType       = bdType;
		memcpy(pPeer->bd, bd, BD_ADDR_SIZE);

		if (pBlueAPI_DS->firstPeerIdx == PEER_IDX_UNUSED)
		{
			pBlueAPI_DS->firstPeerIdx = (uint8_t)peerIdx;
		}
		else
		{
			pPeer = &pBlueAPI_DS->pPeers[pBlueAPI_DS->lastPeerIdx];
			pPeer->nextPeerIdx = (uint8_t)peerIdx;
		}

		pBlueAPI_DS->lastPeerIdx  = (uint8_t)peerIdx;
		pBlueAPI_DS->changed      = TRUE;
	}

	return peerIdx;
} /* blueAPIStore_FindAllocatePeer */

/**
* @brief		blueapi store find allocate enetry
*
* @param	pBlueAPI_DS:
* @param	peerIdx:  
* @param	dataType: 
* @param	allocate: 
* @param	pCompareFunc
* @param	pCompareFuncData
*
* @return
*
*/
PBlueAPI_DSEntry blueAPIStore_FindAllocateEntry(PBlueAPI_DS pBlueAPI_DS, uint8_t peerIdx, uint8_t dataType, BOOL allocate,
                                                       DataStoreEntryCompareFunc pCompareFunc, PVOID pCompareFuncData)
{
	uint16_t            i;
	PBlueAPI_DSEntry pEntry;
	uint16_t            entrySize;
	uint16_t            entryCount;
	PBlueAPI_DSEntry pFirstUnused = NULL;

	switch (dataType)
	{
	case DEVICE_DATA_TYPE_GATT_CCC_BITS: 
	case DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE:
		pEntry      = (PBlueAPI_DSEntry)pBlueAPI_DS->pEntriesS;
		entrySize   = BLUE_API_DATASTORE_SMALL_ENTRY_SIZE;
		entryCount  = pBlueAPI_DS->entriesSCount;
		break;

	case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL: 
	case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
	/* (F_BT_LE_PRIVACY_MODE) */
	case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE:
	/* (F_BT_LE_PRIVACY_RESOLVING) */
	/* (F_BT_LE_DATA_SIGNING) */

	case DEVICE_DATA_TYPE_BREDR_LINKKEY:
		pEntry      = (PBlueAPI_DSEntry)pBlueAPI_DS->pEntriesL;
		entrySize   = BLUE_API_DATASTORE_LARGE_ENTRY_SIZE;
		entryCount  = pBlueAPI_DS->entriesLCount;
		break;

	default: 
		return NULL;
	}

	for (i = 0; i < entryCount; i++)
	{
		/* found matching peer & keyType */
		if ((pEntry->peerIdx == peerIdx) &&
		    (pEntry->dataType == dataType)
		   )
		{
			/* if present, execute compare function */
			if (pCompareFunc != NULL)
			{
				uint16_t comp = pCompareFunc(pEntry, pCompareFuncData);
				if (comp & dataStoreComp_NoMatch_Changed)
				{
					pBlueAPI_DS->changed = TRUE;
				}
				if (comp & dataStoreComp_Match)
				{
					return pEntry;
				}
			}
			else
			{
				return pEntry;
			}
		}
		/* first free entry */
		else if (allocate &&
		         (pEntry->peerIdx == PEER_IDX_UNUSED) &&
		         (pFirstUnused == NULL)
		        )
		{
			pFirstUnused = pEntry;
		}

	/* next entry */
	pEntry = (PBlueAPI_DSEntry)((uint8_t *)pEntry +
	                            offsetof(TBlueAPI_DSEntry, data) +
	                            entrySize);
	}

	if (allocate && (pFirstUnused != NULL))
	{
		memset(pFirstUnused, 0x00, offsetof(TBlueAPI_DSEntry, data) + entrySize);
		pFirstUnused->peerIdx   = peerIdx;
		pFirstUnused->dataType  = dataType;
		pFirstUnused->dataLen   = (uint8_t)entrySize;
		pBlueAPI_DS->changed    = TRUE;
	}

	return pFirstUnused;
} /* blueAPIStore_FindAllocateEntry */

/**
* @brief  blueapi check free peer
*
* @param  pBlueAPI_DS:blueapi DataStore
* @param  peerIdx:
* @param  force:
*
* @return  
*
*/
void blueAPIStore_CheckFreePeer(PBlueAPI_DS pBlueAPI_DS, uint8_t peerIdx, BOOL force)
{
	PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[peerIdx];
	BOOL            found = FALSE;
	uint16_t            i;

	if (pPeer->used != 0x01)
	{
		return;
	}
	
	for (i = 0; i < pBlueAPI_DS->entriesLCount; i++)
	{
		PBlueAPI_DSEntryL pEntry = &pBlueAPI_DS->pEntriesL[i];

		if (pEntry->peerIdx == peerIdx)
		{
			if (force)
			{
				memset(pEntry, 0x00, sizeof(TBlueAPI_DSEntryL));
				pEntry->peerIdx = PEER_IDX_UNUSED;
			}

			/* check if this is a encryption key (bonding) */
			switch (pEntry->dataType)
			{
			case DEVICE_DATA_TYPE_BREDR_LINKKEY:
			case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
			case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:

				/* (F_BT_LE_PRIVACY_MODE) */
				found = TRUE;
				break;

			default:
				break;
			}
		}
	}

	for (i = 0; i < pBlueAPI_DS->entriesSCount; i++)
	{
		PBlueAPI_DSEntryS pEntry = &pBlueAPI_DS->pEntriesS[i];

		if (pEntry->peerIdx == peerIdx)
		{
			/* delete CCC bits if no bonding exists or if forced */
			if (!found || force)
			{
				memset(pEntry, 0x00, sizeof(TBlueAPI_DSEntryS));
				pEntry->peerIdx = PEER_IDX_UNUSED;
			}
		}
	}

	/* free peer only if no entry is using it */
	if (!found || force)
	{
		/* adjust "first" pointer (oldest bond) */
		if (pBlueAPI_DS->firstPeerIdx == peerIdx)
		{
			pBlueAPI_DS->firstPeerIdx = pPeer->nextPeerIdx;
		}

		/* adjust "next" pointer of previous entry */
		for (i = 0; i < pBlueAPI_DS->peerCount; i++)
		{
			PBlueAPI_DSPeer pPrev = &pBlueAPI_DS->pPeers[i];

			if (pPrev->nextPeerIdx == peerIdx)
			{
				pPrev->nextPeerIdx = pPeer->nextPeerIdx;

				/* adjust "last" pointer (newest bond) */
				if (pBlueAPI_DS->lastPeerIdx == peerIdx)
				{
					pBlueAPI_DS->lastPeerIdx = (uint8_t)i;
				}
				break;
			}
		}

		pPeer->used           = 0x00;
		pPeer->nextPeerIdx    = PEER_IDX_UNUSED;
		pBlueAPI_DS->changed  = TRUE;
	}
} /* blueAPIStore_CheckFreePeer */

/**
* @brief  blueapi clear all nvdata
*
* @param  pBlueAPIdata:
*
* @return  
*
*/
void blueAPIStore_ClearAll(PBlueAPI_Data pBlueAPIdata)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t        i;

	memset(pBlueAPI_DS->pNVData, 0x00, blueAPIStore_GetNVDataSize(pBlueAPIdata));

	for (i = 0; i < pBlueAPI_DS->peerCount; i++)
	{
		pBlueAPI_DS->pPeers[i].nextPeerIdx = PEER_IDX_UNUSED;
	}

	for (i = 0; i < pBlueAPI_DS->entriesSCount; i++)
	{
		pBlueAPI_DS->pEntriesS[i].peerIdx = PEER_IDX_UNUSED;
	}

	for (i = 0; i < pBlueAPI_DS->entriesLCount; i++)
	{
		pBlueAPI_DS->pEntriesL[i].peerIdx = PEER_IDX_UNUSED;
	}

	pBlueAPI_DS->maxPeerCount = pBlueAPI_DS->peerCount;
	pBlueAPI_DS->firstPeerIdx = PEER_IDX_UNUSED;
	pBlueAPI_DS->lastPeerIdx  = PEER_IDX_UNUSED;
	pBlueAPI_DS->changed      = TRUE;
} /* blueAPIStore_ClearAll */

/**
* @brief		blueapi store find allocate peer
*			if allocate is true, replace old
*
* @param	pBlueAPIdata:
* @param	bd: 
* @param	bdType
* @param	allocate
*
* @return
*
*/
uint8_t blueAPIStore_FindAllocatePeer_Replace(PBlueAPI_Data pBlueAPIdata, TBdAddr bd, uint8_t bdType, BOOL allocate)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t        peerIdx;

	peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS, bd, bdType, allocate);
	if ((peerIdx == PEER_IDX_UNUSED) && allocate)
	{
		/* remove oldest bond */
		if (pBlueAPI_DS->firstPeerIdx != PEER_IDX_UNUSED)
		{
			PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[pBlueAPI_DS->firstPeerIdx];
			/* TODO: privacy mode not complete: do not replace local IRK dummy bond */
			blueAPI_Send_AuthResultInd(
			                         pPeer->bd,
			                         (TBlueAPI_RemoteBDType)pPeer->bdType,
			                         0, NULL,
			                         blueAPI_LinkKeyTypeDeleted,
			                         blueAPI_CauseSuccess
			                         );

			/* remove oldest bond */
			blueAPIStore_CheckFreePeer(pBlueAPI_DS, pBlueAPI_DS->firstPeerIdx, TRUE);
		}

		peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS, bd, bdType, TRUE);
	}

	return (uint8_t)peerIdx;
} /* blueAPIStore_FindAllocatePeer_Replace */

/**
* @brief		blueapi store compare attribute(CCC) Handle
*
* @param	pBlueAPIdata:
* @param	bd: 
* @param	bdType
* @param	allocate
*
* @return
*
*/
TBlueAPI_DSComp blueAPIStore_CompareCCCHandle(PBlueAPI_DSEntry pEntry, PVOID pData)
{
	PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pFoundBits = (PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)pEntry->data;
	PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pStoreBits = (PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)pData;

	/* compare attribute handle */
	if (pFoundBits->attHandle == pStoreBits->attHandle)
	{
		return dataStoreComp_Match;
	}
	else
	{
		return dataStoreComp_NoMatch;
	}
} /* blueAPIStore_CompareCCCHandle */

/**
* @brief		blueapi store srv change set flag
*
* @param	pEntry:
* @param	pData
*
* @return
*
*/
TBlueAPI_DSComp blueAPIStore_SrvChgSetFlag(PBlueAPI_DSEntry pEntry, PVOID pData)
{
	((PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE)pEntry->data)->indicate = 1;

	/* continue search, mark store as changed */
	return dataStoreComp_NoMatch_Changed;
} /* blueAPIStore_SrvChgSetFlag */

/**
* @brief		blueapi store compare srv change handle
*
* @param	pEntry:
* @param	pData
*
* @return
*
*/
TBlueAPI_DSComp blueAPIStore_CompareSrvChgHandle(PBlueAPI_DSEntry pEntry, PVOID pData)
{
	PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE pFound =
	                           (PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE)pEntry->data;
	PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE pStore =
	                           (PDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE)pData;

	if ((pStore->attHandle == 0) || (pFound->attHandle == pStore->attHandle))
	{
		return dataStoreComp_Match;
	}
	else
	{
		return dataStoreComp_NoMatch;
	}
}

/**
* @brief		blueapi store get data type size
*
* @param	dataType
*
* @return
*
*/
uint8_t blueAPIStore_GetDataTypeSize(uint8_t dataType)
{
	switch (dataType)
	{
	case DEVICE_DATA_TYPE_GATT_CCC_BITS:
		return sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS);
		
	case DEVICE_DATA_TYPE_GATT_CCC_REV_BITS: 
		return sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV);
		
	case DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE:
		return sizeof(TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE);
		
	case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL: 
	case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
		return sizeof(TDEVICE_DATA_ELEMENT_SECMAN_LTK);

	case DEVICE_DATA_TYPE_SECMAN_IRK_LOCAL: 
	case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE: 
		return sizeof(TDEVICE_DATA_ELEMENT_SECMAN_IRK);

	case DEVICE_DATA_TYPE_BREDR_LINKKEY:
		return sizeof(TDEVICE_DATA_ELEMENT_BREDR_LINKKEY);

	default: 
		return 0;
	}
} /* blueAPIStore_GetDataTypeSize */

/**
* @brief		blueapi store delete entry
*
* @param	pEntry
* @param	pData
*
* @return
*
*/
TBlueAPI_DSComp blueAPIStore_DeleteEntry(PBlueAPI_DSEntry pEntry, PVOID pData)
{
	pEntry->peerIdx = PEER_IDX_UNUSED;

	/* continue search, mark store as changed */
	return dataStoreComp_NoMatch_Changed;
} /* blueAPIStore_DeleteEntry */

/****************************************************************************/
/* blueAPIStore_Handle_BT_DEVICE_DATA_SET_IND                               */
/****************************************************************************/
/*
 * set all CCC bits of client:
 * ===========================
 * - DEVICE_DATA_SET_IND from GATT
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
 *   restartHandle = 0x0000 (no more INDs) OR x (will be returned in SET_RESP)
 *   elementCount = 0 <= x <= DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX (0 => delete)
 *   data in p.cccBits[]
 *
 * - DEVICE_DATA_SET_RESP from STORE
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
 *   restartHandle = copy of SET_IND value
 *   elementCount = 0
 *
 * delete all CCC bits for all clients:
 * ====================================
 * - DEVICE_DATA_SET_IND from GATT
 *   bd/bdType = 00:00:00:00:00:00
 *   dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
 *   restartHandle = 0x0000
 *   elementCount = 0
 *
 * - DEVICE_DATA_SET_RESP from STORE
 *   bd/bdType == 00:00:00:00:00:00
 *   dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
 *   restartHandle = 0x0000
 *   elementCount = 0
 *
 * set SRVCHG bits client:
 * ===========================
 * - DEVICE_DATA_SET_IND from GATT
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
 *   restartHandle = 0x0000
 *   elementCount = 0 <= x <= 1 (0 => delete)
 *   data in p.srvChg
 *
 * - DEVICE_DATA_SET_RESP from STORE
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
 *   restartHandle = 0x0000
 *   elementCount = 0
 *
 * set SRVCHG bits of all clients:
 * ===============================
 * - DEVICE_DATA_SET_IND from GATT
 *   bd/bdType = 00:00:00:00:00:00
 *   dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
 *   restartHandle = 0x0000
 *   elementCount = 1
 *   data in p.srvChg
 *
 * - DEVICE_DATA_SET_RESP from STORE
 *   bd/bdType == 00:00:00:00:00:00
 *   dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
 *   restartHandle = 0x0000
 *   elementCount = 0
 *
 * set LTK/IRK/CSRK keys for peer:
 * ===============================
 * - DEVICE_DATA_SET_IND from SECMAN
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_SECMAN_*
 *   restartHandle = 0x0000
 *   elementCount = 0 <= x <= 1 (0 => delete)
 *   data in p.brKey/leLTK/leIRK/leCSRK
 *
 * - DEVICE_DATA_SET_RESP from STORE
 *   bd/bdType != 0
 *   dataType = DEVICE_DATA_TYPE_SECMAN_*
 *   restartHandle = 0x0000
 *   elementCount = 0
 */


/**
* @brief		handle device data set indicate
*	 		set all CCC bits of client:
*			===========================
*			- DEVICE_DATA_SET_IND from GATT
*   				bd/bdType != 0
*   				dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   				restartHandle = 0x0000 (no more INDs) OR x (will be returned in SET_RESP)
*   				elementCount = 0 <= x <= DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX (0 => delete)
*   				data in p.cccBits[]
*
* 			- DEVICE_DATA_SET_RESP from STORE
*   				bd/bdType != 0
*   				dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   				restartHandle = copy of SET_IND value
*   				elementCount = 0
*
* 			delete all CCC bits for all clients:
* 			====================================
* 			- DEVICE_DATA_SET_IND from GATT
*   				bd/bdType = 00:00:00:00:00:00
*   				dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   				restartHandle = 0x0000
*   				elementCount = 0
*
* 			- DEVICE_DATA_SET_RESP from STORE
*   				bd/bdType == 00:00:00:00:00:00
*   				dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   				restartHandle = 0x0000
*   				elementCount = 0
*
* 			set SRVCHG bits client:
* 			===========================
* 			- DEVICE_DATA_SET_IND from GATT
*   				bd/bdType != 0
*   				dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   				restartHandle = 0x0000
*   				elementCount = 0 <= x <= 1 (0 => delete)
*   				data in p.srvChg
*
* 			- DEVICE_DATA_SET_RESP from STORE
*   				bd/bdType != 0
*  				dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   				restartHandle = 0x0000
*   				elementCount = 0
*
* 			set SRVCHG bits of all clients:
* 			===============================
* 			- DEVICE_DATA_SET_IND from GATT
*   				bd/bdType = 00:00:00:00:00:00
*   				dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   				restartHandle = 0x0000
*   				elementCount = 1
*   				data in p.srvChg
*
* 			- DEVICE_DATA_SET_RESP from STORE
*   				bd/bdType == 00:00:00:00:00:00
*   				dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   				restartHandle = 0x0000
*   				elementCount = 0
*
* 			set LTK/IRK/CSRK keys for peer:
* 			===============================
* 			- DEVICE_DATA_SET_IND from SECMAN
*   				bd/bdType != 0
*   				dataType = DEVICE_DATA_TYPE_SECMAN_*
*   				restartHandle = 0x0000
*   				elementCount = 0 <= x <= 1 (0 => delete)
*   				data in p.brKey/leLTK/leIRK/leCSRK
*
* 			- DEVICE_DATA_SET_RESP from STORE
*   				bd/bdType != 0
*   				dataType = DEVICE_DATA_TYPE_SECMAN_*
*   				restartHandle = 0x0000
*   				elementCount = 0
*
* @param	pBlueAPIdata
* @param	pDeviceDataInd
*
* @return
*
*/
BOOL blueAPIStore_Handle_BT_DEVICE_DATA_SET_IND(PBlueAPI_Data    pBlueAPIdata,
                                                       PDEVICE_DATA_IND pDeviceDataInd)
{
	PBlueAPI_DS       pBlueAPI_DS = &pBlueAPIdata->DataStore;
	PDEVICE_DATA      pDeviceData = &pDeviceDataInd->deviceData;
	BOOL              sendResponse = TRUE;
	uint16_t              status = BTSEC_SUCCESS;
	int               iStart, iEnd;
	uint16_t              i, peerIdx;
	BOOL              allocatePeer;
	BOOL              nullBD;
	TBdAddr           BD0    = { 0, 0, 0, 0, 0, 0 };

	nullBD = (memcmp(pDeviceData->bd, BD0, sizeof(TBdAddr)) == 0);

	if (pBlueAPIdata->ConfigParameter.StoreBondMode == blueAPI_StoreBondModeExtStore)
	{
		switch (pDeviceData->dataType)
		{
		case DEVICE_DATA_TYPE_GATT_CCC_BITS:
		{
			TBlueAPI_GATTStoreOpCode opCode = blueAPI_GATTStoreOpSetCCCBits;

			if (nullBD && pDeviceData->elementCount == 0)
			{
				opCode = blueAPI_GATTStoreOpDeleteAllCCCBits;
			}
			else if (pDeviceData->elementCount > 0)
			{
				/* filter out CCC bits with value 0 */

				PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pCCCsrc = pDeviceData->p.cccBit;
				PDEVICE_DATA_ELEMENT_GATT_CCC_BITS pCCCdst = pDeviceData->p.cccBit;

				for (i = 0; i < pDeviceData->elementCount; i++)
				{
					if (pCCCsrc->cccBits != 0x0000)
					{
						/* only move if there is a gap */
						if (pCCCsrc != pCCCdst)
						{
							memcpy(pCCCdst, pCCCsrc, sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS));
						}
						pCCCdst++;
					}
					else
					{
						/* pCCCdst not increased (create gap), adjust Count */
						pDeviceData->elementCount--;
					}
					pCCCsrc++;
				}
			}
			blueAPI_Send_GATTServerStoreInd(
			                              opCode,
			                              pDeviceData->bd,
			                              (TBlueAPI_RemoteBDType)pDeviceData->bdType,
			                              pDeviceData->restartHandle,
			                              (uint8_t)(pDeviceData->elementCount *
			                                     sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS)),
			                              pDeviceData->p.data
			                              );
		}
			break;

		case DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE:
		{
			TBlueAPI_GATTStoreOpCode opCode = blueAPI_GATTStoreOpSetSrvChg;
			uint8_t                     length = 0;

			if (nullBD)
			{
				opCode = blueAPI_GATTStoreOpSetSrvChgFlag;
			}
			else if (pDeviceData->elementCount == 1)
			{
				length = sizeof(TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE);
			}

			blueAPI_Send_GATTServerStoreInd(
			                              opCode,
			                              pDeviceData->bd,
			                              (TBlueAPI_RemoteBDType)pDeviceData->bdType,
			                              pDeviceData->restartHandle,
			                              length,
			                              pDeviceData->p.data
			                              );
		}
			break;

		case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
		case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
		case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE:
			blueAPI_Send_AuthResultInd(
			               pDeviceData->bd,
			               (TBlueAPI_RemoteBDType)pDeviceData->bdType,
			               blueAPIStore_GetDataTypeSize(pDeviceData->dataType),
			               pDeviceData->p.data,
			               (TBlueAPI_LinkKeyType)pDeviceData->dataType,
			               blueAPI_CauseSuccess
			               );
			break;
		default:
		    break;
		}

		return FALSE; /* wait for GATTServerStoreConf / AuthResultConf */
	}

	/* check if this key is allowed to allocate a new peer */
	switch (pDeviceData->dataType)
	{
	case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
	case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
		/* do not allocate a peer for deletes (elementCount == 0) */
		allocatePeer = (pDeviceData->elementCount > 0);
		break;

	default:
		allocatePeer = FALSE;
		break;
	}

	peerIdx = blueAPIStore_FindAllocatePeer_Replace(pBlueAPIdata,
	                                              pDeviceData->bd,
	                                              pDeviceData->bdType,
	                                              allocatePeer
	                                              );

	/* peer not found / no free entry found */
	if (peerIdx == PEER_IDX_UNUSED)
	{
		if (!nullBD ||
		    ((pDeviceData->dataType != DEVICE_DATA_TYPE_GATT_CCC_BITS) &&
		     (pDeviceData->dataType != DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE))
		   )
		{
			status = (allocatePeer) ? BTSEC_ERR_RESOURCE_ERROR : BTSEC_ERR_ITEM_NOT_FOUND;
		}
		else
		{
			/* BD=0 special:                                  */
			/* dataType=DEVICE_DATA_TYPE_GATT_CCC_BITS:       */
			/*    delete all CCC bits for all BDs ..          */
			/* dataType=DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE: */
			/*    set indication flag for all BDs ..          */
			pDeviceData->elementCount = 0;
		}
	}

	if ( nullBD )
	{
		/* operation performed for all BDs */
		iStart = 0;
		iEnd   = pBlueAPI_DS->peerCount - 1;
	}
	else
	{
		iStart = peerIdx;
		iEnd   = peerIdx;
	}

	if ( status != BTSEC_SUCCESS )
	{
		/* send -rsp ... */
	}
	/*--- CCC bits / ServiceChanged characteristic handle/indication flag ---*/
	else if ((pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS) ||
	       (pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE)
	      )
	{
		if (pDeviceData->elementCount > 0)
		{
			/* store listed CCC bits / handle/indication flag */
			uint8_t *                    element;
			uint8_t                      elementLength = sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS);
			DataStoreEntryCompareFunc pCompareFunc  = blueAPIStore_CompareCCCHandle;

			if (pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE)
			{
				elementLength = sizeof(TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE);
				pCompareFunc  = blueAPIStore_CompareSrvChgHandle;
			}

			for (element = pDeviceData->p.data;
			   element < pDeviceData->p.data + (pDeviceData->elementCount * elementLength);
			   element += elementLength)
			{
				PBlueAPI_DSEntry pEntry;

				pEntry = blueAPIStore_FindAllocateEntry(pBlueAPI_DS,
				                                        (uint8_t)peerIdx,
				                                        pDeviceData->dataType,
				                                        TRUE,
				                                        pCompareFunc,
				                                        element
				                                        );
				if (pEntry != NULL)
				{
					/* compare entry before NVRAM update */
					if (memcmp(pEntry->data, element, pEntry->dataLen) != 0)
					{
						/* delete zeroed CCC bits */
						if ((pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS) &&
						    (((PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)element)->cccBits == 0x0000)
						   )
						{
							pEntry->peerIdx   = PEER_IDX_UNUSED;
						}
						else
						{
							pEntry->peerIdx   = (uint8_t)peerIdx;
							pEntry->dataType  = pDeviceData->dataType;
							pEntry->dataLen   = elementLength;
							memcpy(pEntry->data, element, pEntry->dataLen);
						}

						pBlueAPI_DS->changed = TRUE;
					}
				}
				else
				{
					status = BTSEC_ERR_RESOURCE_ERROR;
				}
		  	}
		}
		else
		{
			DataStoreEntryCompareFunc pCompareFunc;

			if ((pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE) &&
			  nullBD
			 )
			{
				/* dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE */
				/* set indication flags for all peers               */
				pCompareFunc = blueAPIStore_SrvChgSetFlag;
			}
			else
			{
				/* delete all CCC bits */
				pCompareFunc = blueAPIStore_DeleteEntry;
			}

			for (i = iStart; i <= iEnd; i++)
			{
				if (pBlueAPI_DS->pPeers[i].used == 0x01)
				{
					blueAPIStore_FindAllocateEntry(pBlueAPI_DS,
					                             (uint8_t)i,
					                             pDeviceData->dataType,
					                             FALSE,
					                             pCompareFunc,
					                             NULL
					                             );
				}
			}
		}
	}
	else
	{
		PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[peerIdx];

		if (pDeviceData->elementCount == 1)
		{
			/* store link key */
			PBlueAPI_DSEntry pEntry;

			switch (pDeviceData->dataType)
			{
			case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
			case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
				blueAPI_Send_AuthResultInd(
				                         pPeer->bd,
				                         (TBlueAPI_RemoteBDType)pPeer->bdType,
				                         sizeof(TDEVICE_DATA_ELEMENT_SECMAN_LTK),
				                         pDeviceData->p.data,
				                         (TBlueAPI_LinkKeyType)pDeviceData->dataType,
				                         blueAPI_CauseSuccess
				                         );

				/* wait for AuthResultConf */
				sendResponse = FALSE;
				break;

			default:
				break;
			}

			pEntry = blueAPIStore_FindAllocateEntry(pBlueAPI_DS,
			                                      (uint8_t)peerIdx,
			                                      pDeviceData->dataType,
			                                      TRUE,
			                                      NULL,
			                                      NULL
			                                      );

			if (pEntry != NULL)
			{
				pEntry->dataLen = blueAPIStore_GetDataTypeSize(pDeviceData->dataType);
				memcpy(pEntry->data, pDeviceData->p.data, pEntry->dataLen);
				pBlueAPI_DS->changed = TRUE;
			}
			else
			{
				status = BTSEC_ERR_RESOURCE_ERROR;
			}
		}
		else if ((pDeviceDataInd->deviceData.dataType == DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL) ||
		         (pDeviceDataInd->deviceData.dataType == DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE)
		        )
		{
			blueAPI_Send_AuthResultInd(
			                         pPeer->bd,
			                         (TBlueAPI_RemoteBDType)pPeer->bdType,
			                         0, /* linkKeyLength */
			                         NULL, /* linkKey */
			                         blueAPI_LinkKeyTypeDeleted,
			                         blueAPI_CauseSuccess
			                         );

			/* delete all link keys */
			blueAPIStore_FindAllocateEntry(pBlueAPI_DS,
			                             (uint8_t)peerIdx,
			                             pDeviceData->dataType,
			                             FALSE,
			                             blueAPIStore_DeleteEntry,
			                             NULL
			                             );
		}
	}

	/* on delete: check and remove unused peer entries */
	if ((status == BTSEC_SUCCESS) && (pDeviceData->elementCount == 0))
	{
		for (i = iStart; i <= iEnd; i++)
		{
			blueAPIStore_CheckFreePeer(pBlueAPI_DS, (uint8_t)i, FALSE);
		}
	}

	blueAPIStore_Save(pBlueAPIdata);

	if (sendResponse)
	{
		pDeviceData->elementCount = 0;

		btApiDEVICE_DATA_SET_RESP(pDeviceDataInd->handle, status, pDeviceData);
		return TRUE; /* free message */
	}

	return FALSE; /* wait for AuthResultConf */
} /* blueAPIStore_Handle_BT_DEVICE_DATA_SET_IND */

/**
* @brief  
* @detail
* 	get all CCC bits of client:
* 	===========================
* 		- DEVICE_DATA_GET_IND from GATT
*   		bd/bdType != 0
*  		dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   		restartHandle = 0x0000 (first GET_IND) OR x (from previous GET_RESP)
*   		elementCount = 0
*
* 		- DEVICE_DATA_GET_RESP from STORE
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   		restartHandle = x (more values, use in next GET_IND) OR 0x0000 (no more values)
*   		elementCount = 0 <= x <= DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX
*   		data in p.cccBits[]
*
* 	get all BDs that set this CCC bit:
* 	==================================
* 		- DEVICE_DATA_GET_IND from GATT
*   		bd/bdType = 00:00:00:00:00:00
*   		dataType = DEVICE_DATA_TYPE_GATT_CCC_BITS
*   		restartHandle = 0x0000 (first GET_IND) OR x (from previous GET_RESP)
*   		elementCount = 1
*   		data in p.cccBits[0]
*
* 		- DEVICE_DATA_GET_RESP from STORE
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_GATT_CCC_REV_BITS
*   		restartHandle = x (more values, use in next GET_IND) OR 0x0000 (no more values)
*   		elementCount = 0 <= x <= DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV_MAX
*   		data in p.cccBitsRev[]
*
* 	get SRVCHG bits of client:
* 	===========================
* 		- DEVICE_DATA_GET_IND from GATT
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   		restartHandle = 0x0000
*   		elementCount = 0
*
* 		- DEVICE_DATA_GET_RESP from STORE
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE
*   		restartHandle = 0x0000
*   		elementCount = 0 <= x <= 1
*   		data in p.srvChg
*
* 	get LTK/IRK/CSRK keys for peer:
*	===============================
* 		- DEVICE_DATA_GET_IND from SECMAN
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_SECMAN_*
*   		restartHandle = 0x0000
*   		elementCount = 0
*
* 		- DEVICE_DATA_GET_RESP from STORE
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_SECMAN_*
*   		restartHandle = 0x0000
*   		elementCount = 0 <= x <= 1
*   		data in p.brKey/leLTK/leIRK/leCSRK
*
* 	get all IRK keys:
* 	=================
* 		- DEVICE_DATA_GET_IND from SECMAN
*   		bd/bdType = 00:00:00:00:00:00
*   		dataType = DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE
*   		restartHandle = 0x0000 (first GET_IND) OR x (from previous GET_RESP)
*   		elementCount = 0
*
* 		- DEVICE_DATA_GET_RESP from STORE
*   		bd/bdType != 0
*   		dataType = DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE
*   		restartHandle = x (more values, use in next GET_IND) OR 0x0000 (no more values)
*   		elementCount = 0 <= x <= 1
*   		data in p.leIRK
*
* @param  pBlueAPIdata:
* @param  pMsg
*
* @return  
*
*/
BOOL blueAPIStore_Handle_BT_DEVICE_DATA_GET_IND(PBlueAPI_Data    pBlueAPIdata,
                                                       PDEVICE_DATA_IND pDeviceDataInd)
{
	PBlueAPI_DS       pBlueAPI_DS = &pBlueAPIdata->DataStore;
	PDEVICE_DATA      pDeviceData = &pDeviceDataInd->deviceData;
	uint16_t              status      = BTSEC_SUCCESS;
	uint16_t              peerIdx;
	BOOL              nullBD;
	TBdAddr           BD0         = { 0, 0, 0, 0, 0, 0 };

	nullBD = (memcmp(pDeviceData->bd, BD0, sizeof(TBdAddr)) == 0);

	if (pBlueAPIdata->ConfigParameter.StoreBondMode == blueAPI_StoreBondModeExtStore)
	{
		/* no results */
		pDeviceData->elementCount = 0;

		switch (pDeviceData->dataType)
		{
		case DEVICE_DATA_TYPE_GATT_CCC_BITS:
			{
				TBlueAPI_GATTStoreOpCode opCode = blueAPI_GATTStoreOpGetCCCBits;

				if (nullBD)
				{
					opCode                          = blueAPI_GATTStoreOpGetAllCCCBits;
					pBlueAPIdata->dsCCCSearchHandle = pDeviceData->p.cccBit[0].attHandle;
				}

				blueAPI_Send_GATTServerStoreInd(
				                              opCode,
				                              pDeviceData->bd,
				                              (TBlueAPI_RemoteBDType)pDeviceData->bdType,
				                              pDeviceData->restartHandle,
				                              0, NULL
				                              );
			}
			break;

		case DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE:
			blueAPI_Send_GATTServerStoreInd(
			                                blueAPI_GATTStoreOpGetSrvChg,
			                                pDeviceData->bd,
			                                (TBlueAPI_RemoteBDType)pDeviceData->bdType,
			                                pDeviceData->restartHandle,
			                                0, NULL
			                                );
			break;

		case DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL:
		case DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE:
		/* (F_BT_LE_PRIVACY_MODE) */
		case DEVICE_DATA_TYPE_SECMAN_IRK_REMOTE:
		/* (F_BT_LE_PRIVACY_RESOLVING) */
		/* (F_BT_LE_DATA_SIGNING) */
			blueAPI_Send_AuthResultRequestInd(
			                                  pDeviceData->bd,
			                                  (TBlueAPI_RemoteBDType)pDeviceData->bdType,
			                                  (TBlueAPI_LinkKeyType)pDeviceData->dataType,
			                                  pDeviceData->restartHandle
			                                  );
			break;

		default:
			break;
		}

		return FALSE; /* wait for GATTServerStoreConf / AuthResultConf */
	}

	if (nullBD)
	{
		/* BD=NULL: reverse search for for BDs for given handle/CCC bits */
		peerIdx = 0xFFFF;
	}
	else
	{
		peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS,
		                                        pDeviceData->bd,
		                                        pDeviceData->bdType,
		                                        FALSE
		                                        );
	}

	/* no results */
	pDeviceData->elementCount = 0;

	/* peer not found */
	if (peerIdx == PEER_IDX_UNUSED)
	{
		status = BTSEC_ERR_ITEM_NOT_FOUND;
	}
	else if (pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS)
	{
		uint16_t i;

		if (nullBD)
	    {
			uint16_t attHandle = pDeviceData->p.cccBit[0].attHandle;

			for (i = pDeviceData->restartHandle; i < pBlueAPI_DS->entriesSCount; i++)
			{
				PBlueAPI_DSEntryS pEntry = &pBlueAPI_DS->pEntriesS[i];

				/* matching entry */
				if ((pEntry->peerIdx != PEER_IDX_UNUSED) &&
				    (pEntry->dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS) &&
				    (((PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)pEntry->data)->attHandle == attHandle)
				   )
				{
					if (pDeviceData->elementCount < DEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV_MAX)
					{
						PDEVICE_DATA_ELEMENT_GATT_CCC_BITS_REV pCCCRev;

						pCCCRev = &pDeviceData->p.cccBitRev[pDeviceData->elementCount++];
						pCCCRev->bdType = pBlueAPI_DS->pPeers[pEntry->peerIdx].bdType;
						memcpy(pCCCRev->bd, pBlueAPI_DS->pPeers[pEntry->peerIdx].bd, BD_ADDR_SIZE);

						pDeviceData->restartHandle = 0x0000;
					}
					else
					{
						pDeviceData->restartHandle = i;
						break;
					}
				}
			}

			/* adjust response type */
			pDeviceData->dataType = DEVICE_DATA_TYPE_GATT_CCC_REV_BITS;
    	}
	    else
	    {
			for (i = pDeviceData->restartHandle; i < pBlueAPI_DS->entriesSCount; i++)
			{
				PBlueAPI_DSEntryS pEntry = &pBlueAPI_DS->pEntriesS[i];

				/* matching entry */
				if ((pEntry->peerIdx == peerIdx) &&
				    (pEntry->dataType == DEVICE_DATA_TYPE_GATT_CCC_BITS)
				   )
				{
					if (pDeviceData->elementCount < DEVICE_DATA_ELEMENT_GATT_CCC_BITS_MAX)
					{
						memcpy(&pDeviceData->p.cccBit[pDeviceData->elementCount++],
						       pEntry->data,
						       sizeof(TDEVICE_DATA_ELEMENT_GATT_CCC_BITS)
						       );

						pDeviceData->restartHandle = 0x0000;
					}
					else
					{
						pDeviceData->restartHandle = i;
						break;
					}
				}
			}
	    }
  	}
	else if (pDeviceData->dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE)
	{
		uint16_t i;

		for (i = 0; i < pBlueAPI_DS->entriesSCount; i++)
		{
			PBlueAPI_DSEntryS pEntry = &pBlueAPI_DS->pEntriesS[i];

			/* matching entry (only 1 per BD/peerIdx can exist!) */
			if ((pEntry->peerIdx == peerIdx) &&
			  (pEntry->dataType == DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE)
			 )
			{
				memcpy(&pDeviceData->p.srvChg,
				       pEntry->data,
				       sizeof(TDEVICE_DATA_ELEMENT_GATT_SRV_CHG_HANDLE)
				       );

				pDeviceData->elementCount = 1;
				break;
			}
		}
	}
	else if ((pDeviceData->dataType & DEVICE_DATA_TYPE_MASK) == DEVICE_DATA_TYPE_SECMAN)
	{

		if (nullBD)
		{
			uint16_t i;

			for (i = pDeviceData->restartHandle; i < pBlueAPI_DS->entriesLCount; i++)
			{
				PBlueAPI_DSEntryL pEntry = &pBlueAPI_DS->pEntriesL[i];

				if ((pEntry->peerIdx != PEER_IDX_UNUSED) &&
				    (pEntry->dataType == pDeviceData->dataType)
				   )
				{
					if (pDeviceData->elementCount == 0)
					{
						pDeviceData->elementCount   = 1;
						pDeviceData->restartHandle  = 0x0000;
						pDeviceData->bdType = pBlueAPI_DS->pPeers[pEntry->peerIdx].bdType;
						memcpy(pDeviceData->bd, pBlueAPI_DS->pPeers[pEntry->peerIdx].bd, BD_ADDR_SIZE);
						memcpy(pDeviceData->p.data, pEntry->data, pEntry->dataLen);
					}
					else
					{
						pDeviceData->restartHandle  = i;
						break;
					}
				}
		  	}
		}
		else
		/* (F_BT_LE_PRIVACY_RESOLVING) */
		{
			PBlueAPI_DSEntry pEntry;

			pEntry = blueAPIStore_FindAllocateEntry(pBlueAPI_DS,
			                                      (uint8_t)peerIdx,
			                                      pDeviceData->dataType,
			                                      FALSE,
			                                      NULL,
			                                      NULL
			                                      );
			if (pEntry != NULL)
			{
				pDeviceData->elementCount = 1;
				memcpy(pDeviceData->p.data, pEntry->data, pEntry->dataLen);
			}
		}
	}

	if (pDeviceData->elementCount == 0)
	{
		status = BTSEC_ERR_ITEM_NOT_FOUND;
	}

	btApiDEVICE_DATA_GET_RESP(pDeviceDataInd->handle, status, pDeviceData);
	return TRUE; /* free message */
} /* blueAPIStore_Handle_BT_DEVICE_DATA_GET_IND */

/**
* @brief  blueapi queue in msg to pBlueAPIdata->DataStoreQueue
*
* @param  pBlueAPIdata:
* @param  pMsg
*
* @return  
*
*/
BOOL blueAPIStore_QueueIn(PBlueAPI_Data pBlueAPIdata, LPblueFaceMsg pMsg)
{
	uint16_t i;

	for (i = 0; i < 4; i++)
	{
		PBlueAPI_BufferElement pElement = &pBlueAPIdata->DSBufferElement[i];

		if (!pElement->used)
		{
			pElement->used  = TRUE;
			pElement->lpMsg = (PVOID)pMsg;

			osQueueIn(&pBlueAPIdata->DataStoreQueue, (PVOID)pElement);
			return FALSE; /* message queued, do not release */
		}
	}

	return TRUE; /* release message */
} /* blueAPIStore_QueueIn */

/**
* @brief  blueapi trigger data store queue
*
* @param  pBlueAPIdata:
* @param  releaseMsg: if release previous message
*
* @return  
*
*/
void blueAPIStore_QueueTrigger(PBlueAPI_Data pBlueAPIdata, BOOL releaseMsg)
{
	LPblueFaceMsg     pMsg;
	PDEVICE_DATA_IND  pDeviceDataInd;
	BOOL              retVal = TRUE;

	/* release previous message */
	if (releaseMsg && (pBlueAPIdata->pDSCurrentMsg != NULL))
	{
		osBufferRelease((uint8_t *)pBlueAPIdata->pDSCurrentMsg);
		pBlueAPIdata->pDSCurrentMsg = NULL;
	}

	while (pBlueAPIdata->pDSCurrentMsg == NULL)
	{
		PBlueAPI_BufferElement pElement = osQueueOut(&pBlueAPIdata->DataStoreQueue);
		if (pElement == NULL)
		{
			break;
		}

		pMsg            = (LPblueFaceMsg)(pElement->lpMsg);
		pDeviceDataInd  = &pMsg->p.deviceDataIndication;
		pElement->used  = FALSE;

		switch (pMsg->command)
		{
		case BT_DEVICE_DATA_GET_IND: 
			retVal = blueAPIStore_Handle_BT_DEVICE_DATA_GET_IND(pBlueAPIdata, pDeviceDataInd);
			break;

		  case BT_DEVICE_DATA_SET_IND: /*---------------------------------------*/
		    retVal = blueAPIStore_Handle_BT_DEVICE_DATA_SET_IND(pBlueAPIdata, pDeviceDataInd);
		    break;

		  default:  /*----------------------------------------------------------*/
		    break;
		}

		if (retVal)
		{
			osBufferRelease((uint8_t *)pMsg);
		}
		else
		{
			/* keep message for response from application */
			pBlueAPIdata->pDSCurrentMsg = pMsg;
		}
	}
} /* blueAPIStore_QueueTrigger */

/**
* @brief  blueapi store change mode
*
* @param  pBlueAPIdata:
* @param  storeBondMode
* @param  storeBondSize
* @param  init: weather init
*
* @return  
*
*/
void blueAPIStore_ChangeMode(PBlueAPI_Data            pBlueAPIdata,
                             TBlueAPI_StoreBondModes  storeBondMode,
                             uint16_t                     storeBondSize,
                             BOOL                     init)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t        i;

	if ((pBlueAPIdata->ConfigParameter.StoreBondMode == storeBondMode) &&
	  (pBlueAPI_DS->maxPeerCount == storeBondSize) &&
	  !init
	 )					/*if init and not change, just return*/
	{
		return;
	}

	BLUEAPI_TRACE_PRINTF_5(BLUEAPI_TRACE_MASK_TRACE,
	                        "blueAPIStore_ChangeMode(mode:%d->%d size:%d->%d init:%d)",
	                        pBlueAPIdata->ConfigParameter.StoreBondMode, storeBondMode,
	                        pBlueAPI_DS->maxPeerCount, storeBondSize,
	                        init
	                        );

	pBlueAPIdata->ConfigParameter.StoreBondMode = storeBondMode;

	if (storeBondMode != blueAPI_StoreBondModeNVStore)
	{
		blueAPIStore_ClearAll(pBlueAPIdata);
		pBlueAPI_DS->maxPeerCount = (uint8_t)storeBondSize;
	}
	else
	{
		/* load from nv storage and verify checksum */
		if (!blueAPIStore_Load(pBlueAPIdata))
		{
			blueAPIStore_ClearAll(pBlueAPIdata);
			pBlueAPI_DS->maxPeerCount = (uint8_t)storeBondSize;
		}
		/* change maxPeerCount */
		else if (!init)
		{
			/* if maxPeerCount is reduced, delete all peers */
			if (pBlueAPI_DS->maxPeerCount > storeBondSize)
			{
				blueAPIStore_ClearAll(pBlueAPIdata);
				pBlueAPI_DS->maxPeerCount = (uint8_t)storeBondSize;
			}
			else if (pBlueAPI_DS->maxPeerCount != storeBondSize)
			{
				pBlueAPI_DS->maxPeerCount = (uint8_t)storeBondSize;
				pBlueAPI_DS->changed      = TRUE;
			}
		}
	}

	/* remove entries without peers */
	for (i = 0; i < pBlueAPI_DS->entriesSCount; i++)
	{
		PBlueAPI_DSEntryS pEntry = &pBlueAPI_DS->pEntriesS[i];
		if (pEntry->peerIdx != PEER_IDX_UNUSED)
		{
			PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[pEntry->peerIdx];
			if (pPeer->used != 0x01)
			{
				pEntry->peerIdx       = PEER_IDX_UNUSED;
				pBlueAPI_DS->changed  = TRUE;
			}
		}
	}

	/* remove entries without peers */
	for (i = 0; i < pBlueAPI_DS->entriesLCount; i++)
	{
		PBlueAPI_DSEntryL pEntry = &pBlueAPI_DS->pEntriesL[i];
		if (pEntry->peerIdx != PEER_IDX_UNUSED)
		{
			PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[pEntry->peerIdx];
			if (pPeer->used != 0x01)
			{
				pEntry->peerIdx       = PEER_IDX_UNUSED;
				pBlueAPI_DS->changed  = TRUE;
			}
		}
	}

	/* remove peers without entries */
	for (i = 0; i < pBlueAPI_DS->peerCount; i++)
	{
		blueAPIStore_CheckFreePeer(pBlueAPI_DS, (uint8_t)i, FALSE);
	}

	blueAPIStore_Save(pBlueAPIdata);
} /* blueAPIStore_ChangeMode */

/**
* @brief  delete peer in nv
*
* @param  pBlueAPIdata
* @param  pBD
* @param  bdType
*
* @return  
*
*/
BOOL blueAPIStore_DeletePeer(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	TBdAddr     BD0         = { 0, 0, 0, 0, 0, 0 };
	uint16_t        peerIdx;

	if (memcmp(pBD, BD0, sizeof(TBdAddr)) == 0)
	{
		pBD = NULL;
	}

	for (peerIdx = 0; peerIdx < pBlueAPI_DS->peerCount; peerIdx++)
	{
		PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[peerIdx];

		if ((pPeer->used == 0x01) &&
		    ((bdType == BLUEFACE_BDTYPE_ANY) || (pPeer->bdType == bdType)) &&
		    ((pBD == NULL) || (memcmp(pPeer->bd, pBD, BD_ADDR_SIZE) == 0))
		   )
		{
			/* remove peer and all it's entries */
			blueAPIStore_CheckFreePeer(pBlueAPI_DS, (uint8_t)peerIdx, TRUE);
		}
	}

	return blueAPIStore_Save(pBlueAPIdata);
} /* blueAPIStore_DeletePeer */

/**
* @brief		blueapi store br link key
*
* @param	pBlueAPIdata:
* @param	pBD: 
* @param	pElement
*
* @return
*
*/
BOOL blueAPIStore_SetBRLinkKey(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD,
                               PDEVICE_DATA_ELEMENT_BREDR_LINKKEY pElement)
{
	PBlueAPI_DS       pBlueAPI_DS = &pBlueAPIdata->DataStore;
	PBlueAPI_DSEntry  pEntry;
	uint16_t              peerIdx;

	peerIdx = blueAPIStore_FindAllocatePeer_Replace(pBlueAPIdata,
	                                              pBD,
	                                              BLUEFACE_BDTYPE_BR_EDR,
	                                              (pElement->keyType != BLUEFACE_KEYTYPE_DELETE)
	                                              );
	if (peerIdx == PEER_IDX_UNUSED)
	{
		/* not an error for key delete */
		return (pElement->keyType == BLUEFACE_KEYTYPE_DELETE);
	}

	pEntry = blueAPIStore_FindAllocateEntry(pBlueAPI_DS, (uint8_t)peerIdx,
	                                      DEVICE_DATA_TYPE_BREDR_LINKKEY,
	                                      TRUE, NULL, NULL);

	if (pEntry != NULL)
	{
		if (pElement->keyType != BLUEFACE_KEYTYPE_DELETE)
		{
			pEntry->dataLen = sizeof(TDEVICE_DATA_ELEMENT_BREDR_LINKKEY);
			memcpy(pEntry->data, (PVOID)pElement, pEntry->dataLen);
			pBlueAPI_DS->changed = TRUE;
		}
		else
		{
			pEntry->peerIdx = PEER_IDX_UNUSED;
		}
	}

	if ((pEntry == NULL) || (pEntry->peerIdx == PEER_IDX_UNUSED))
	{
		blueAPIStore_CheckFreePeer(pBlueAPI_DS, (uint8_t)peerIdx, FALSE);
	}

	return blueAPIStore_Save(pBlueAPIdata);
} /* blueAPIStore_SetBRLinkKey */

/**
* @brief		get br link key from nv
*
* @param	pBlueAPIdata:
* @param	pBD:  
* @param	pElement
*
* @return
*
*/
BOOL blueAPIStore_GetBRLinkKey(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, PDEVICE_DATA_ELEMENT_BREDR_LINKKEY pElement)
{
	PBlueAPI_DS       pBlueAPI_DS = &pBlueAPIdata->DataStore;
	PBlueAPI_DSEntry  pEntry;
	uint16_t              peerIdx;

	peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS, pBD, BLUEFACE_BDTYPE_BR_EDR, FALSE);
	if (peerIdx == PEER_IDX_UNUSED)
	{
		return FALSE;
	}

	pEntry = blueAPIStore_FindAllocateEntry(pBlueAPI_DS, (uint8_t)peerIdx,
	                                      DEVICE_DATA_TYPE_BREDR_LINKKEY,
	                                      FALSE, NULL, NULL);
	if (pEntry == NULL)
	{
		return FALSE;
	}

	memcpy((PVOID)pElement, pEntry->data, pEntry->dataLen);
	return TRUE;
} /* blueAPIStore_GetBRLinkKey */

/**
* @brief		set peer info to nv
*
* @param	pBlueAPIdata:
* @param	pBD:  
* @param	bdType: 
* @param	pDeviceName: 
* @param	nameLength
* @param	pAppData
*
* @return
*
*/
BOOL blueAPIStore_SetPeerInfo(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD,
                              uint8_t bdType, uint8_t * pDeviceName, uint16_t nameLength,
                              uint32_t* pAppData)
{
	PBlueAPI_DS     pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t            peerIdx;
	PBlueAPI_DSPeer pPeer;

	peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS, pBD, bdType, FALSE);
	if (peerIdx == PEER_IDX_UNUSED)
	{
		return FALSE;
	}

	pPeer = &pBlueAPI_DS->pPeers[peerIdx];

	/* copy device data */
	if (pDeviceName != NULL)
	{
		if (nameLength >= BLUE_API_DEVICE_NAME_LENGTH)
		{
			nameLength = BLUE_API_DEVICE_NAME_LENGTH -1;
		}

		if (memcmp(pPeer->deviceName, pDeviceName, nameLength) != 0)
		{
			memcpy(pPeer->deviceName, pDeviceName, nameLength);
			pPeer->deviceName[nameLength] = '\0';
			pBlueAPI_DS->changed = TRUE;
		}
	}

	return blueAPIStore_Save(pBlueAPIdata);
} /* blueAPIStore_SetPeerInfo */

/**
* @brief		get peer info from nv
*
* @param	pBlueAPIdata:
* @param	pBD:  
* @param	bdType: 
* @param	pDeviceName: 
* @param	nameLength
* @param	pAppData
*
* @return
*
*/
BOOL blueAPIStore_GetPeerInfo(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD,
                              uint8_t bdType, uint8_t * pDeviceName, uint16_t nameLength,
                              uint32_t* pAppData)
{
	PBlueAPI_DS     pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint16_t            peerIdx;
	PBlueAPI_DSPeer pPeer;

	peerIdx = blueAPIStore_FindAllocatePeer(pBlueAPI_DS, pBD, bdType, FALSE);
	if (peerIdx == PEER_IDX_UNUSED)
	{
		return FALSE;
	}

	pPeer = &pBlueAPI_DS->pPeers[peerIdx];

	/* copy devicename */
	if (pDeviceName != NULL)
	{
		uint16_t copyLen = strlen((char *)pPeer->deviceName);

		if (copyLen >= nameLength)
		{
			copyLen = nameLength -1;
		}

		memcpy(pDeviceName, pPeer->deviceName, copyLen);
	}

	/* copy app data */
	if (pAppData != NULL)
	{
		*pAppData = pPeer->appData;
	}

	return TRUE;
} /* blueAPIStore_GetPeerInfo */

/**
* @brief  send auth list
*
* @param  pBlueAPIdata
* @param  pBD
* @param  bdType
*
* @return  
*
*/
BOOL blueAPIStore_SendAuthList(PBlueAPI_Data pBlueAPIdata, uint8_t * pBD, uint8_t bdType)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;
	uint8_t        peerIdx     = pBlueAPI_DS->firstPeerIdx;
	BOOL        found       = FALSE;
	TBdAddr     BD0         = { 0, 0, 0, 0, 0, 0 };
	uint16_t        i;

	if (memcmp(pBD, BD0, BD_ADDR_SIZE) == 0)	/*if check bdaddr*/
	{
		pBD   = NULL;
		found = TRUE;
	}

	while (peerIdx != PEER_IDX_UNUSED)
	{
		PBlueAPI_DSPeer pPeer = &pBlueAPI_DS->pPeers[peerIdx];

		if ((pPeer->used == 0x01) &&
		    ((bdType == BLUEFACE_BDTYPE_ANY) || (pPeer->bdType == bdType)) &&
		    ((pBD == NULL) || (memcmp(pPeer->bd, pBD, BD_ADDR_SIZE) == 0))
		   )
		{
			for (i = 0; i < pBlueAPI_DS->entriesLCount; i++)
			{
				PBlueAPI_DSEntryL pEntry = &pBlueAPI_DS->pEntriesL[i];

				if (pEntry->peerIdx == peerIdx)
				{
					if ((pPeer->bdType == BLUEFACE_BDTYPE_BR_EDR) &&
					  (pEntry->dataType == DEVICE_DATA_TYPE_BREDR_LINKKEY)
					 )
					{
						PDEVICE_DATA_ELEMENT_BREDR_LINKKEY pKey;

						pKey = (PDEVICE_DATA_ELEMENT_BREDR_LINKKEY)&pEntry->data;

						blueAPI_Send_AuthListInfo(
						                          pPeer->bd,
						                          (TBlueAPI_RemoteBDType)pPeer->bdType,
						                          (TBlueAPI_LinkKeyType)pKey->keyType,
						                          pPeer->appData,
						                          pPeer->deviceName
						                          );
						found = TRUE;
					}
					else if ((pPeer->bdType & BLUEFACE_BDTYPE_LE_MASK) &&
					       ((pEntry->dataType == DEVICE_DATA_TYPE_SECMAN_LTK_LOCAL) ||
					       (pEntry->dataType == DEVICE_DATA_TYPE_SECMAN_LTK_REMOTE)
					      ))
					{
						blueAPI_Send_AuthListInfo(
						                          pPeer->bd,
						                          (TBlueAPI_RemoteBDType)pPeer->bdType,
						                          (TBlueAPI_LinkKeyType)pEntry->dataType,
						                          pPeer->appData,
						                          pPeer->deviceName
						                          );
						found = TRUE;
					}
				}
			}
		}

		peerIdx = pPeer->nextPeerIdx;
	}

	return found;
} /* blueAPIStore_SendAuthList */

/**
* @brief  get local copy of NVRAM data etc.
*
* @param  pBlueAPIdata:
*
* @return  
*
*/
void blueAPIStore_Init(PBlueAPI_Data pBlueAPIdata)
{
	PBlueAPI_DS pBlueAPI_DS = &pBlueAPIdata->DataStore;

	BLUEAPI_TRACE_PRINTF_4(BLUEAPI_TRACE_MASK_TRACE,
	                        "blueAPIStore_Init nvSize=0x%x (%d/%d/%d)",
	                        blueAPIStore_GetNVDataSize(pBlueAPIdata),
	                        pBlueAPI_DS->peerCount,
	                        pBlueAPI_DS->entriesSCount,
	                        pBlueAPI_DS->entriesLCount
	                        );

	/* init with default parameters */
	blueAPIStore_ChangeMode(pBlueAPIdata,
	                      (TBlueAPI_StoreBondModes)pBlueAPIdata->ConfigParameter.StoreBondMode,
	                      pBlueAPIdata->ConfigParameter.StoreBondSize,
	                      TRUE
	                      );
} /* blueAPIStore_Init */
