/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattutil.c
* @brief     GATT (Bluetooth Generic Attribute Protocol) utilities.
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#include <flags.h>
#include <os_pool.h>
#include <os_mem.h>
#include <att.h>
#include <gattdef.h>
#include <btcommon.h>
#include <sdp_code.h>
#include <blueapi_api.h>
#include <upper_stack_global.h>

#define TRACE_MODULE_ID     MID_BT_GATT
#if (GATT_NO_BUILTIN_SERVICE)
extern char cGattDeviceName[];
#else

/**
 * @brief  service definition.
 *
 *	 GAP service and GATT service
 */
const TAttribAppl gattBuiltinProfile[] =
{
	/** Primary Service */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE | ATTRIB_FLAG_BR_EDR),  /**< wFlags     */

		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(UUID_GAP),                      /**< service UUID */
			HI_WORD(UUID_GAP)
		},
		UUID_16BIT_SIZE,                            /**< bValueLen     */
		NULL,                                       /**< pValueContext */
		GATT_PERM_READ                              /**< wPermissions  */
	},
	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                     /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** characteristic value */
	{
		(ATTRIB_FLAG_VOID | ATTRIB_FLAG_ASCII_Z),   /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_DEVICE_NAME),
			HI_WORD(GATT_UUID_CHAR_DEVICE_NAME)
		},
		0,                                          /**< bValueLen = strlen()+1 */
		(PVOID)gGattDeviceName,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                     /**< characteristic properties */
		/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/* characteristic value */
	{
		ATTRIB_FLAG_VOID,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_APPEARANCE),
			HI_WORD(GATT_UUID_CHAR_APPEARANCE),
		},
		2,                                          /**< bValueLen */
		(PVOID)gGattAppearance,
		GATT_PERM_READ                              /* wPermissions */
	},
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ                       /* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* wPermissions */
    },
    /* characteristic value */   //XXXXMJMJ configurable thru API ???
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_ASCII_Z),   /* wFlags */
        {                                           /* bTypeValue */
            LO_WORD(GATT_UUID_CHAR_PER_PREF_CONN_PARAM),
            HI_WORD(GATT_UUID_CHAR_PER_PREF_CONN_PARAM)
        },
        8,                                          /* bValueLen */
        (PVOID)gGattPerPrefConnParam,
        GATT_PERM_READ                              /* wPermissions */
    }
};
#endif

/**
* @brief		check if attribute has specific type (16 bit UUID)
*
* @param	pAttrib
* @param	wUUID
*
* @return
*
*/
BOOL  gattAttribIsType16( PAttrib pAttrib, uint16_t wUUID )
{
	return ( !(pAttrib->wFlags & ATTRIB_FLAG_UUID_128BIT)   &&
	        (LE_EXTRN2WORD(pAttrib->TypeValue.s.bType) == wUUID)
	     );
}

/**
* @brief		check if attribute is service declaration
*
* @param	pAttrib
*
* @return
*
*/
BOOL  gattAttribIsService( PAttrib  pAttrib )
{
	return( gattAttribIsType16( pAttrib, GATT_UUID_PRIMARY_SERVICE ) ||
	      gattAttribIsType16( pAttrib, GATT_UUID_SECONDARY_SERVICE )
	    );
}

/**
* @brief		check if attribute is client characteristic configuration
*
* @param	pAttrib
*
* @return
*
*/
BOOL  gattAttribIsCharClientCfg( PAttrib pAttrib )
{
 	return ( gattAttribIsType16( pAttrib, GATT_UUID_CHAR_CLIENT_CONFIG ) );
}

/**
* @brief		check if attribute is characteristic declaration
*
* @param	pAttrib
*
* @return
*
*/
BOOL  gattAttribIsCharDecl( PAttrib pAttrib )
{
	return ( gattAttribIsType16( pAttrib, GATT_UUID_CHARACTERISTIC ) );
}

/**
* @brief		 check if attribute may be accessed over current transport link type
*
* @param	wHandle: 
* @param	pAttrib 
* @param	pRClient
*
* @return
*
*/
BOOL  gattAttribAccessAllowed( uint16_t wHandle,
                                        PAttrib pAttrib, PGATTRClient pRClient )
{
	PAttrib pAttribSrvDecl;
	BOOL    LE      = (pRClient->pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK);
	BOOL    Allowed = TRUE;

	/* get associated service declaration attribute */
	if ( !(pAttrib->wFlags & ATTRIB_FLAG_UUID_128BIT) &&
	   (LE_EXTRN2WORD(pAttrib->TypeValue.s.bType) == GATT_UUID_PRIMARY_SERVICE)
	 )
	{
		pAttribSrvDecl = pAttrib;
	}
	else
	{
		pAttribSrvDecl = gattGoToAttribute( -1, wHandle,
	                                      GATT_UUID_PRIMARY_SERVICE, NULL );
	}

	if ( pAttribSrvDecl != (PAttrib)0 )
	{
		if ( (!LE && !(pAttribSrvDecl->wFlags & ATTRIB_FLAG_BR_EDR)) ||
		     (LE && !(pAttribSrvDecl->wFlags & ATTRIB_FLAG_LE))
		   )
		{
			Allowed = FALSE;
		}
	}

	return ( Allowed );
}

/**
* @brief		check attribute read/write permissions and matching transport link type
*
* @param	pRClient: 
* @param	wHandle
* @param	pAttrib 
* @param	Write
*
* @return
*
*/
uint32_t  gattAttribCheckPermissions( PGATTRClient pRClient,
                                     uint16_t wHandle, PAttrib pAttrib, BOOL Write )
{
	uint8_t  bPerm      = 0;
	int   iAuthorReq = 0;
	uint32_t dwATTError = ATT_OK;

	/* GATT TS 4.0.1 test cases TP/GAR/SR/BI-34-C ... :                   */
	/* check if accessed over current link type (LE or BR/EDR) is allowed */
	if ( gattAttribAccessAllowed( wHandle, pAttrib, pRClient ) )
	{
		if ( Write )
		{
			iAuthorReq = GATT_PERM_WRITE_AUTHOR_GET(pAttrib->wPermissions);
			bPerm      = GATT_PERM_WRITE_AUTHEN_GET(pAttrib->wPermissions);
			if ( (iAuthorReq == 0) && (bPerm == GATT_PERM_NONE) )
			{
				dwATTError  = ATT_ERR_WRITE_NOT_PERMITTED;
			}
		}
		else
		{
			/* read */
			iAuthorReq = GATT_PERM_READ_AUTHOR_GET(pAttrib->wPermissions);
			bPerm      = GATT_PERM_READ_AUTHEN_GET(pAttrib->wPermissions);
			if ( (iAuthorReq == 0) && (bPerm == GATT_PERM_NONE) )
			{
				dwATTError  = ATT_ERR_READ_NOT_PERMITTED;
			}
		}
	}
	else
	{
		/* XXXXMJMJ there is no appropriate error code !!?? how can applic. */
		/* distinguish this from profile dependend codes??                  */
		/* test cases TP/GAR/SR/BI-34-C .. expect "an <Application Error> code" */
		dwATTError = ATT_ERR_MIN_APPLIC_CODE;
	}

	if ( dwATTError == ATT_OK )
	{
		/* check for authentication, authorization, encryption .. */
		switch( bPerm )
		{
		default:
		case GATT_PERM_ALL:
			/* no authentication required */
			break;
		case GATT_PERM_AUTHEN_REQ:
			if ( !(pRClient->bSecurityFlags & GATT_RCLIENT_SEC_AUTHEN) )
			{
				dwATTError = ATT_ERR_INSUFFICIENT_AUTHENTICATION;
			}
			break;
		case GATT_PERM_AUTHEN_MITM_REQ:
			if ( !(pRClient->bSecurityFlags & GATT_RCLIENT_SEC_AUTHEN_MITM) )
			{
				dwATTError = ATT_ERR_INSUFFICIENT_AUTHENTICATION;
			}
			break;
		}

		if ( dwATTError == ATT_OK )
		{
			int  iKeysize;

			/* check encryption key size: */
			GATT_PERM_KEYSIZE_GET(pAttrib->wPermissions, iKeysize);
			if ( iKeysize > pRClient->bEncyptKeysize )
			{
				dwATTError = ATT_ERR_INSUFFICIENT_KEY_SIZE;
			}

			if ( dwATTError == ATT_OK )
			{
				/* authorization can be checked by application only: */
				if ( iAuthorReq && !(pAttrib->wFlags & ATTRIB_FLAG_VALUE_APPL) )
				{
					dwATTError = ATT_ERR_INSUFFICIENT_AUTHORIZATION;
				}
			}
		}
	}

	return( dwATTError );
}

/**
* @brief  go to attribute with given UUID (16 bit) starting at current attribute.
* 
* @param wHandle: handle of start attribute
* @param iDirection:
*   			< 0          backward
*   			= 0          convert wHandle into attribute pointer (no move)
*   			> 0          forward
* @param pwDstHandle:handle of the destination attribute (for iDirection != 0)
*
* @return  pAttrib      pointer of the destination attribute
*
*/
PAttrib gattGoToAttribute( int iDirection, uint16_t wHandle,
                                             uint16_t wUUID, uint16_t * pwDstHandle )
{
	PGATTService pGATTService;
	int          iSrvDescr;        /* index of descriptor */
	int          iAttribIdx = 0;
	PAttrib      pAttrib = NULL;

	/* convert wHandle into pointer of starting attribute */
	iSrvDescr = GATT_ATTR_HANDLE_NBR_GET(wHandle);
	//if ( iSrvDescr < BT_GATT_SERVER_MAX_SERVICES_COUNT )
    if ( iSrvDescr < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count )    
	{
		pGATTService = &pGATT->pServer->pService[iSrvDescr];
		if ( pGATTService->wUsed )
		{
			iAttribIdx = GATT_ATTR_HANDLE_IDX_GET(wHandle);

			if ( iAttribIdx < pGATTService->wAttribCnt )
			{
				pAttrib = pGATTService->pAttribFirst + iAttribIdx;
			}
		}

		if ( pAttrib == NULL )
		{
		  /* illegal wHandle */
		}
		else
		{
			if ( iDirection != 0 )
			{
				int  iDelta;
				uint16_t w;

				if ( iDirection > 0 )
				{
					iDelta = 1;
				}
				else
				{
					iDelta = -1;
				}

				iAttribIdx += iDelta;
				while ( (iAttribIdx >= 0) &&
				        (iAttribIdx < pGATTService->wAttribCnt)
				      )
				{
					pAttrib = pGATTService->pAttribFirst + iAttribIdx;

					if ( !(pAttrib->wFlags & ATTRIB_FLAG_UUID_128BIT) )
					{
						w = LE_EXTRN2WORD(pAttrib->TypeValue.s.bType);
						if ( w == wUUID )
						{
							/* found */
							if ( pwDstHandle != NULL )
							{
								*pwDstHandle = pGATTService->wHandleStart + iAttribIdx;
							}
							break;
						}
					    else
					    {
							/* don't cross certain boundaries depending on search UUID */
							if ( (w == GATT_UUID_PRIMARY_SERVICE)   ||
							   (w == GATT_UUID_SECONDARY_SERVICE) ||
							   ((wUUID != GATT_UUID_PRIMARY_SERVICE) &&
							                              (w == GATT_UUID_CHARACTERISTIC))
							 )
							{
								pAttrib = NULL;
								break;
							}
					    }
					}

				  	iAttribIdx += iDelta;
				}
			}
		}
	}

	return( pAttrib );
}

/**
* @brief  convert handle to attribute pointer
* 
* @param wHandle: handle for convert
*
* @return  
*
*/
PAttrib gattHandleToAttribute( uint16_t wHandle )
{
	return( gattGoToAttribute( 0, wHandle, 0, NULL ) );
}

#if (GATT_CONSECUTIVE_ATTRIB_HANDLES)

/**
* @brief  convert handle to service number or index in service attribute array
* 
* @param ToNbr: True: return servce number in service attribute array
*				False: return wHandle - startHandle(index?)
* @param wHandle: handle for convert
*
* @return  index or service number
*
*/
int gattHandleToServiceNbrIdx( BOOL ToNbr, uint16_t wHandle )
{
	int          i;
	PGATTService pGATTService;
	PGATTServer  pServer = &pGATT->SrvCl.Server;

	for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++ )
	{
		pGATTService = &pServer->pService[i];
		if ( pGATTService->wUsed )
		{
            if (otp_str_data.gEfuse_UpperStack_s.bqb_en == 0)
            {
    			if ( (wHandle >= pGATTService->wHandleStart) &&
    			   (wHandle <= (pGATTService->wHandleStart + pGATTService->wAttribCnt - 1))
    			 )
    			{
    				if ( ToNbr )
    				{
    					return( pGATTService->wNbr );
    				}
    				else
    				{
    					return( wHandle - pGATTService->wHandleStart );
    				}
    			}
            }
            else    /* bqb mode */
            {
                if (wHandle <= pGATTService->wHandleStart)
                {
                    if ( ToNbr )
                    {
                        return ( pGATTService->wNbr );
                    }
                    else
                    {
                        return 0;
                    }
                }
                else if ((wHandle >= pGATTService->wHandleStart)
                         && (wHandle <= (pGATTService->wHandleStart + pGATTService->wAttribCnt - 1)))
                {
                    if ( ToNbr )
                    {
                        return ( pGATTService->wNbr );
                    }
                    else
                    {
                        return ( wHandle - pGATTService->wHandleStart );
                    }
                }
            }
		}
	}

	if ( ToNbr )
	{
		//return( BT_GATT_SERVER_MAX_SERVICES_COUNT );
		return(otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count);
	}
	else
	{
		return( -1 );
	}
}
#endif  /* (GATT_CONSECUTIVE_ATTRIB_HANDLES) */

/**
* @brief  convert attribute pointer to handle
*
* @param  pAttrib
*
* @return  
*
*/

uint16_t gattAttributeToHandle( PAttrib pAttrib )
{
	PGATTServer pServer;
	PAttrib     pAttribLast;
	int         i;
	uint32_t       dwHandle = 0;

	pServer = &pGATT->SrvCl.Server;
	//for ( i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++ )
    for ( i = 0; i <otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++ )
	{
		if ( pServer->pService[i].wUsed != 0 )
		{
			pAttribLast = pServer->pService[i].pAttribFirst +
			                               (pServer->pService[i].wAttribCnt-1);
			if ( (pAttrib >= pServer->pService[i].pAttribFirst) &&
			(pAttrib <= pAttribLast)
			)
			{
				dwHandle = (uint32_t)pAttrib - (uint32_t)pServer->pService[i].pAttribFirst;
				dwHandle = pServer->pService[i].wHandleStart + dwHandle/sizeof(TAttrib);
				break;
			}
		}
	}

	return( (uint16_t)dwHandle );
}

/**
* @brief  get instantiation of Client Characteristic Confirmation Bits.
*
* @param  DirectAccess: 
* @param  wHandle
* @param  pBdAddr
*
* @return  
*
*/
uint16_t gattCCCBitsGet( BOOL DirectAccess, uint16_t wHandle, LPBdAddr pBdAddr )
{
	PGATTCCCBitsTable  pCCCBitsTable;
	int      i;
	uint16_t     wCCCBits = GATT_CLIENT_CHAR_CONFIG_DEFAULT;

	if ( !DirectAccess )
	{
		uint16_t  wHandleCCCD;

		/* search for CCC bits definition that if present follows value */
		if ( gattGoToAttribute( 1, wHandle,
		                               GATT_UUID_CHAR_CLIENT_CONFIG, &wHandleCCCD ) == NULL )
		{
			wHandle = 0;
		}
		else
		{
			wHandle = wHandleCCCD;
		}
	}

	if ( wHandle != 0 )
	{
		//for ( i = 0; i < BT_GATT_SERVER_MAX_CCC_BITS_COUNT; i++ )  
		for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount; i++ ) 
		{
			pCCCBitsTable = &pGATT->pServer->pCCCBits[i];
			if ( pCCCBitsTable->bUsed )
			{
				if ( (pCCCBitsTable->wHandle == wHandle) &&
				     (memcmp(pCCCBitsTable->Bd, pBdAddr, sizeof(TBdAddr)) == 0)
				   )
				{
					wCCCBits = pCCCBitsTable->wCCCBits;
					break;
				}
			}
		}
	}
	return( wCCCBits );
}

/**
* @brief		get flags of Client Characteristic Confirmation Bits in local table
*
* @param	pL2CChannel: 
* @param	wHandle
*
* @return
*
*/
int gattCCCBitsGetFlags( PGATTL2CChannel pL2CChannel, uint16_t wHandle )
{
	PGATTCCCBitsTable  pCCCBitsTable;
	int      i;

	//for ( i = 0; i < BT_GATT_SERVER_MAX_CCC_BITS_COUNT; i++ )   
    for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount; i++ ) 
    {
		pCCCBitsTable = &pGATT->pServer->pCCCBits[i];
		if ( pCCCBitsTable->bUsed )
		{
			if ( (pCCCBitsTable->wHandle == wHandle) &&
			   (memcmp(pCCCBitsTable->Bd, &pL2CChannel->RemoteBd, sizeof(TBdAddr)) == 0)
			 )
			{
				return( (int)pCCCBitsTable->bFlags );
			}
		}
	}
	return( -1 );
}

/**
* @brief		set instantiation of Client Characteristic Confirmation Bits
*
* @param	pL2CChannel
* @param	Client
* @param	wHandle
* @param	wCCCBits
*
* @return
*
*/
int gattCCCBitsSet( PGATTL2CChannel pL2CChannel,
                                      BOOL Client, uint16_t wHandle, uint16_t wCCCBits )
{
	PGATTCCCBitsTable  pCCCBitsTable, pCCCBitsTableFound;
	int      i;
	BOOL     NewEntry = FALSE;

	//if ( GATT_ATTR_HANDLE_NBR_GET(wHandle) >= BT_GATT_SERVER_MAX_SERVICES_COUNT )
	if (GATT_ATTR_HANDLE_NBR_GET(wHandle) >= otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count )
	{
		/* (nbr. of) associated service not found, return "no change" */
		return( 0 );
	}

	pCCCBitsTableFound = NULL;

	//for ( i = 0; i < BT_GATT_SERVER_MAX_CCC_BITS_COUNT; i++ )
	for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount; i++ )
	{
		pCCCBitsTable = &pGATT->pServer->pCCCBits[i];
		if ( pCCCBitsTable->bUsed )
		{
			if ( (pCCCBitsTable->wHandle == wHandle) &&
			   (memcmp(pCCCBitsTable->Bd, &pL2CChannel->RemoteBd, sizeof(TBdAddr)) == 0)
			 )
			{
				pCCCBitsTableFound = pCCCBitsTable;
				break;
			}
		}
		else
		{
			break;
		}
	}

	if ( pCCCBitsTableFound == NULL )
	{
		/* try to create new entry */
	//	if ( i < BT_GATT_SERVER_MAX_CCC_BITS_COUNT )
        if ( i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount )
		{
			pCCCBitsTableFound   = pCCCBitsTable;
			pCCCBitsTable->bUsed = 1;
			NewEntry             = TRUE;
		}
	}

	if ( pCCCBitsTableFound != NULL )
	{
		/* set Client Characteristic Configuration Bits in local table */
		if ( Client )
		{
			/* client writes CCCD before DEVICE_DATA_RESP msg from BlueAPI/store */
			/* is received, assure that client value will not be overwritten:    */
			if ( !pL2CChannel->Ready )
			{
				pCCCBitsTable->bFlags |= GATT_CCCBITS_CLIENT_WRITE_ONLY;
			}
		}
		else
		{
			if ( pCCCBitsTable->bFlags & GATT_CCCBITS_CLIENT_WRITE_ONLY )
			{
				return( 0 );
			}
		}

		i = 1;
		if ( NewEntry )
		{
			memcpy( pCCCBitsTable->Bd, &pL2CChannel->RemoteBd, sizeof(TBdAddr) );
			pCCCBitsTable->wHandle  = wHandle;
		}
		else
		{
			if ( pCCCBitsTable->wCCCBits == wCCCBits )  /* existing value was not changed */
			{
				i = 0;
			}
		}
		pCCCBitsTable->wCCCBits = wCCCBits;

		return( i );
	}

	return( -1 );
}

/**
* @brief  various operation on/with Client Characteristic Confirmation Bits in
* local table:
*
* @param  Op
* @param  pL2CChannel
* @param  pList
*
* @return  
*
*/
int gattCCCBitsOp( TGATTCCCBitsOp Op,
                                   PGATTL2CChannel pL2CChannel, uint8_t * pList )
{
	PGATTCCCBitsTable                   pCCCBitsTable;
	PDEVICE_DATA_ELEMENT_GATT_CCC_BITS  pElement;
	int      i, iCount;

	iCount = 0;
	pElement = (PDEVICE_DATA_ELEMENT_GATT_CCC_BITS)pList;

	//for ( i = 0; i < BT_GATT_SERVER_MAX_CCC_BITS_COUNT; i++ )
	for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount; i++ )
	{
		pCCCBitsTable = &pGATT->pServer->pCCCBits[i];
		if ( pCCCBitsTable->bUsed )
		{
			if ( memcmp(pCCCBitsTable->Bd, &pL2CChannel->RemoteBd, sizeof(TBdAddr)) == 0 )
			{
				iCount++;
				switch( Op )
				{
				default:
					break;
				case CCCBitsClear:
					pCCCBitsTable->bUsed  = 0;
					pCCCBitsTable->bFlags = 0;
					break;
				case CCCBitsAppend:
					pElement->attHandle = pCCCBitsTable->wHandle;
					pElement->cccBits   = pCCCBitsTable->wCCCBits;
					pElement++;
					break;
				}
			}
		}
	}

	return( iCount );
}

/**
* @brief  change l2cap state
*
* @param  pL2CChannel
* @param  L2CState
*
* @return  
*
*/
void gattL2CChangeState( PGATTL2CChannel pL2CChannel, TGATTL2CState L2CState )
{
	pL2CChannel->L2CState = L2CState;
}

/*---------------------------------------------------------------------------
 * link L2CAP channels to remote and/or local client descriptors.
 *--------------------------------------------------------------------------*/

void gattL2CChannelsLinkToClientDescr()
{
    int               i;
    PGATTL2CChannel   pL2CChannel;

    PGATTServer       pServer = &pGATT->SrvCl.Server;

    pGATT->pServer = pServer;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDon[i];

        /* remote client descriptor */
        pL2CChannel->pRClient              = &pServer->pRClientDon[i];
        pServer->pRClientDon[i].pL2CChannel    = pL2CChannel;

        /* local client descriptor */
        pL2CChannel->pClient               = &pGATT->SrvCl.pClientDon[i];
        pGATT->SrvCl.pClientDon[i].pL2CChannel = pL2CChannel;

        pL2CChannel->pTxData = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTTxData)*otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize);
        
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[i];

        /* remote client descriptor */
        pL2CChannel->pRClient              = &pServer->pRClientDoff[i];
        pServer->pRClientDoff[i].pL2CChannel    = pL2CChannel;

        /* local client descriptor */
        pL2CChannel->pClient               = &pGATT->SrvCl.pClientDoff[i];
        pGATT->SrvCl.pClientDoff[i].pL2CChannel = pL2CChannel;
        pL2CChannel->pTxData = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TGATTTxData)*otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize);

/*			
        if(FALSE == DLPS_BUFFER_REG((UINT8*)&pL2CChannel->pTxData, sizeof(pL2CChannel->pTxData), FALSE))
        {
            GATT_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR,"GATT: DLPS_BUFFER_REG fail");

        }
*/			
    }

}


#if (GATT_USE_LL_WSIZE_EQ_1)
/*---------------------------------------------------------------------------
 * flush L2C channel transmit queue
 *--------------------------------------------------------------------------*/
void gattL2CChannelFlushTxQueue( PGATTL2CChannel pL2CChannel )
{
  PGATTTxData pTxData;
  uint8_t *      pBuffer;

  pTxData = osQueueOut( &pL2CChannel->TxDataQueue );
  while( pTxData != NULL )
  {
    if ( pTxData->TxDataType == TxTypeNotification )
      pBuffer = pTxData->p.AttribUpdData.pBuffer;
    else
      pBuffer = pTxData->p.AttribUpdData.pBuffer;

    osBufferRelease( pBuffer );
    osQueueIn( &pL2CChannel->TxDataQueueFree, pTxData );
    pTxData = osQueueOut( &pL2CChannel->TxDataQueue );
  }
}
#endif

/**
* @brief  gatt allocate l2cap channel
*
* @param  LE: 
*
* @return  
*
*/
PGATTL2CChannel gattL2CChannelAllocate( BOOL LE )
{
	int              i, j;
	PGATTL2CChannel  pL2CChannel;
	PGATTTxData      pTxData;
    BOOL bFound = FALSE;


    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDon[i];

        if ( !pL2CChannel->wUsed )
        {
            GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE, "GATT: gattL2CChannelAllocate on / wNbr=%d", i);
            bFound = TRUE;
            goto TagFound;
        }
    }


    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[i];

        if ( !pL2CChannel->wUsed )
        {
            GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE, "GATT: gattL2CChannelAllocate off / wNbr=%d", i);
            bFound = TRUE;
            break;
        }
    }

TagFound:
    
    if ( bFound)
    {
        memset( &pL2CChannel->wReserved, 0,   /* preserve wNbr and client references */
                    sizeof(TGATTL2CChannel) - offsetof(TGATTL2CChannel, wReserved) );
        pL2CChannel->wUsed = 1;
        if ( LE )
            pL2CChannel->wMTUSizeMax = otp_str_data.gEfuse_UpperStack_s.att_max_mtu_size ;
        else
            pL2CChannel->wMTUSizeMax = BT_US_PDU_L2C_BYTE_COUNT;

        pL2CChannel->wMTUSize    = ATT_MTU_SIZE_LE_DEFAULT;
        pL2CChannel->L2CState    = gattL2CStateIdle;

        /* clear data of associated client descriptors too */

        memset( &pL2CChannel->pClient->bReserved, 0,   /* preserve backward reference */
                sizeof(TGATTClient) - offsetof(TGATTClient, bReserved) );

        memset( &pL2CChannel->pRClient->bReserved, 0,  /* preserve backward reference */
                sizeof(TGATTRClient) - offsetof(TGATTRClient, bReserved) );

        /* init. q of tx data descriptors */
        pTxData = &pL2CChannel->pTxData[0];

        for ( j = 0; j < otp_str_data.gEfuse_UpperStack_s.gatt_max_tx_wsize; j++ )
        {
            osQueueIn( &pL2CChannel->TxDataQueueFree, pTxData );
            pTxData++;
        }
    }
    else
    {
       GATT_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR,
                                "!! GATT: gattL2CChannelAllocate / no L2C chnl");
       return NULL;
    }

	return( pL2CChannel );
}

/**
* @brief  free l2cap channel
*
* @param  pL2CChannel
*
* @return  
*
*/
void gattL2CChannelFree( PGATTL2CChannel pL2CChannel )
{
		GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE,
	                       "GATT: gattL2CChannelFree / wNbr=%d", pL2CChannel->wNbr);

#if (GATT_USE_LL_WSIZE_EQ_1)
	gattL2CChannelFlushTxQueue( pL2CChannel );
#endif

	/* check work if buffer of incomplete remote client transaction has to be freed */
	if ( pL2CChannel->pRClient != NULL )
	{
		if ( pL2CChannel->pRClient->CurrTrans.p.read.pWorkBuffer != NULL )
		{
			osBufferRelease(pL2CChannel->pRClient->CurrTrans.p.read.pWorkBuffer);
			pL2CChannel->pRClient->CurrTrans.p.read.pWorkBuffer = NULL;
		}
		pL2CChannel->pRClient->CurrTrans.wTransType = GATT_TYPE_UNDEFINED;
	}

#if (GATT_CCC_BITS_TO_BTSEC_AT_CONN_END)
	if ( pL2CChannel->pRClient->bCCCBitsModified )
	{
		/* instruct BTSEC to store CCC bits if bond was created/exists */
		gattSendDEVICE_DATA_IND( DEVICE_DATA_SET_IND,
		                    DEVICE_DATA_TYPE_GATT_CCC_BITS, FALSE, pL2CChannel, 0, NULL );
		pL2CChannel->pRClient->bCCCBitsModified = 0;
	}

	if ( pL2CChannel->pRClient->iServiceChangedHandle != 0 )
	{
		/* instruct BTSEC to store/remove ServiceChanged characteristic */
		/* handle if bond was created/exists                            */
		BOOL Delete;
		uint16_t wHandle;

		Delete = (pL2CChannel->pRClient->iServiceChangedHandle > 0) ? FALSE : TRUE;
		if ( Delete )
		{
			wHandle = (uint16_t)(-pL2CChannel->pRClient->iServiceChangedHandle);
		}
		else
		{
			wHandle = (uint16_t)pL2CChannel->pRClient->iServiceChangedHandle;
		}
		gattSendDEVICE_DATA_IND( DEVICE_DATA_SET_IND,
		                             DEVICE_DATA_TYPE_GATT_SRV_CHG_HANDLE, Delete,
		                             pL2CChannel, wHandle, NULL );
		pL2CChannel->pRClient->iServiceChangedHandle = 0;
	}
#endif

	gattDeferredMsgLLFree( pL2CChannel, TRUE );

	if (pL2CChannel == pGATT->pLELockedChannel)
	{
		pGATT->pLELockedChannel = NULL;
	}

	gattL2CChangeState( pL2CChannel, gattL2CStateIdle );

	if ( pL2CChannel->DevDataGetRspPending )
	{
	/* wait for DEVICE_DATA_RESP .. */
	}
	else
	{
		/* delete CCC bits from local table */
		gattCCCBitsOp( CCCBitsClear, pL2CChannel, NULL );
		pL2CChannel->wUsed = 0;
	}

}

#if F_BT_LE_BT41_SUPPORT
uint16_t gattGetLEConnectionHandle(uint16_t cid)
{
    PGATTL2CChannel 	pL2CChannel;
	
	pL2CChannel = gattL2CChannelFindHandle( FALSE, cid);
	if ( pL2CChannel == NULL )
	{
	    return 0;
	}
	else
	{
        return pL2CChannel->wLEConnectionHandle;
	}

}

uint16_t gattGetLEConnectionCid(uint16_t handle)
{
    PGATTL2CChannel 	pL2CChannel;
	
	pL2CChannel = gattL2CChannelFindHandle( TRUE, handle);
	if ( pL2CChannel == NULL )
	{
	    return 0;
	}
	else
	{
        return pL2CChannel->cid;
	}

}
#endif
/**
* @brief  find L2C channel based on handle (cid or wLEConnectionHandle)l
*
* @param  handle
* @param  status
*
* @return  
*
*/
PGATTL2CChannel gattL2CChannelFindHandle( BOOL LE, uint16_t wHandle )
{
	int              i;
	PGATTL2CChannel  pL2CChannel;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDon[i];

        //XXXXMJMJ if GATT_CON_REQ is rejected by GATT pL2CChannel->cid !!
        //XXXXMJMJ GATT_DISC_RESP with cid=0 is not unique for simultaneous
        //XXXXMJMJ failing connection setup !!!!
        if ( pL2CChannel->wUsed &&

                ((!LE && (pL2CChannel->cid == wHandle)) ||
                 (LE && (wHandle != 0) && (pL2CChannel->wLEConnectionHandle == wHandle))
                )

           )
        {
            return ( pL2CChannel );
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[i];

        //XXXXMJMJ if GATT_CON_REQ is rejected by GATT pL2CChannel->cid !!
        //XXXXMJMJ GATT_DISC_RESP with cid=0 is not unique for simultaneous
        //XXXXMJMJ failing connection setup !!!!
        if ( pL2CChannel->wUsed &&

                ((!LE && (pL2CChannel->cid == wHandle)) ||
                 (LE && (wHandle != 0) && (pL2CChannel->wLEConnectionHandle == wHandle))
                )

           )
        {
            return ( pL2CChannel );
        }
    }

    GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
                        "!!gattL2CChannelFindHandle, wHandle(0x%x) not found", wHandle);

	return( NULL );
}

/**
* @brief  find L2C channel based on connection state and "Client Characteristic
* Configuration Bits" allowing indications resp. notifications
*
* @param  wHandle: 
* @param  ppL2CChannel
*
* @return  
*
*/
int  gattL2CChannelFindConnIndNotif( uint16_t wHandle,
                                               PGATTL2CChannel * ppL2CChannel )
{
	int              i, iReturn;
	PGATTL2CChannel  pL2CChannel;

	iReturn       = -1;
	*ppL2CChannel = NULL;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDon[i];

        if ( pL2CChannel->wUsed && (pL2CChannel->L2CState == gattL2CStateConnected) )
        {
            *ppL2CChannel = pL2CChannel;
            iReturn       = gattCCCBitsGet( FALSE, wHandle, &pL2CChannel->RemoteBd );
            return ( iReturn );
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[i];

        if ( pL2CChannel->wUsed && (pL2CChannel->L2CState == gattL2CStateConnected) )
        {
            *ppL2CChannel = pL2CChannel;
            iReturn       = gattCCCBitsGet( FALSE, wHandle, &pL2CChannel->RemoteBd );
            return ( iReturn );
        }
    }

	return( iReturn );
}

/**
* @brief  find L2C channel based on connection state and BD address
*
* @param  pBdAddr: 
*
*
* @return  
*
*/
PGATTL2CChannel  gattL2CChannelFindBD( LPBdAddr pBdAddr )
{
	int              i;
	PGATTL2CChannel  pL2CChannel;

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDon[i];

        if ( pL2CChannel->wUsed &&
                (pL2CChannel->L2CState == gattL2CStateConnected) &&
                (memcmp(pL2CChannel->RemoteBd, pBdAddr, sizeof(TBdAddr)) == 0)
           )
        {
            return ( pL2CChannel );
        }
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pL2CChannel = &pGATT->pL2CChannelDoff[i];

        if ( pL2CChannel->wUsed &&
                (pL2CChannel->L2CState == gattL2CStateConnected) &&
                (memcmp(pL2CChannel->RemoteBd, pBdAddr, sizeof(TBdAddr)) == 0)
           )
        {
            return ( pL2CChannel );
        }
    }

	return( NULL );
}

/**
* @brief		allocate work buffer
*
* @param	iSize
*
* @return
*
*/
int gattWorkBufferAlloc( int iSize )
{
	if ( osBufferGet(BTSystemPoolID, iSize, (PVOID)&pGATT->pWorkBuffer) )
	{

		  GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_ERROR,
		                     "!!GATT: gattWorkBufferAlloc / failed, iSize=0x%x", iSize);

		assert(FALSE);
		return( 0 );
	}

	return( iSize );
}

/**
* @brief		free work buffer
*
*
* @return
*
*/
void gattWorkBufferFree( )
{
	osBufferRelease( pGATT->pWorkBuffer );
	pGATT->pWorkBuffer = NULL;
}

/**
* @brief  check application provided service coding and if service is already
* 		registered
* 
* @param pAttrib
* @param iCnt
*
* @return
*
*/
uint16_t gattServiceCheck( PAttrib pAttrib, int iCnt )
{
	int    i /*, iProperties*/;
	uint16_t   wType16;
	uint16_t   wCause = GATT_SUCCESS;

	if ( (pAttrib == NULL) || (iCnt == 0) )
	{
		return( GATT_ERR_ILLEGAL_PARAMETER );
	}

/* E.g. BAS (Battery) is a candidate for a service that is present
          in multiple instances. */

	for ( i = 0; i < iCnt ; i++ )
	{
		/*iProperties = -1;*/
		if ( !(pAttrib->wFlags & ATTRIB_FLAG_UUID_128BIT) )
		{
			wType16 = LE_EXTRN2WORD(pAttrib->TypeValue.s.bType);

			switch( wType16 )
			{
			case GATT_UUID_CHAR_AGGR_FORMAT:
				/* not yet implemented .. */
				wCause = GATT_ERR_ILLEGAL_CODING;
				break;
            case GATT_UUID_CHAR_CLIENT_CONFIG:
                break;
            default:
                if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
                {
                    if (pAttrib->wPermissions & 0x0070 )
                    {
                        wCause = GATT_ERR_ILLEGAL_CODING;
                    }
                }
                break;
			}
		}
		if ( wCause != GATT_SUCCESS )
		{
			break;
		}

		pAttrib++;
	}

  return( wCause );
}

/**
* @brief  allocate GATT server profile/service descriptor.
*		 built-in services (GAP,GATT) must be allocated first.
* 
*
* @return allocated gatt service
*
*/
PGATTService gattServiceAlloc( )
{
	PGATTServer pServer;
	int         i;

	pServer = &pGATT->SrvCl.Server;
    for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++ )
	{
		if ( pServer->pService[i].wUsed == 0 )
		{
			memset( &pServer->pService[i], 0, sizeof(TGATTService) );
			pServer->pService[i].wNbr  = i;
			pServer->pService[i].wUsed = 1;

			GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE, "GATT: gattServiceAlloc / wNbr=%d", i);

			return( &pServer->pService[i] );
		}
	}

	GATT_TRACE_PRINTF_0(GATT_TRACE_MASK_ERROR, "!!GATT: gattServiceAlloc / failed");

	return( (PGATTService)0 );
}

/**
* @brief  free GATT profile/service descriptor 
*
* @param pGATTService
*
* @return 
*
*/
void gattServiceFree( PGATTService pGATTService )
{
		GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE,
	                    "GATT: gattServiceFree / wNbr=%d", pGATTService->wNbr);
#if F_BT_BREDR_SUPPORT
	pGATTService->Sdp.State = SdpStateReleased;
#endif
	pGATTService->wUsed     = 0;
}

/**
* @brief init gatt srvice 
* 		normally set starthandle and sdp next handle 
*
* @param pGATTService
*
* @return 
*
*/
void gattServiceInit( PGATTService pGATTService )
{
    if (otp_str_data.gEfuse_UpperStack_s.bqb_en == 0)
    {
#if (GATT_CONSECUTIVE_ATTRIB_HANDLES)
    	{
    		int          i;
    		PGATTService pGATTService2;
    		uint16_t         wHandleMax = 0;
    		PGATTServer  pServer    = &pGATT->SrvCl.Server;

    		/* find max handle used so far */
    		for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++ )
    		{
    			pGATTService2 = &pServer->pService[i];
    			if ( (pGATTService2 != pGATTService)&& pGATTService2->wUsed &&
    			   ((pGATTService2->wHandleStart + pGATTService2->wAttribCnt-1) > wHandleMax)
    			 )
    			{
    				wHandleMax = pGATTService2->wHandleStart + pGATTService2->wAttribCnt - 1;
    			}
    		}
    		/*start handle is max+1*/
    		pGATTService->wHandleStart = wHandleMax + 1;
    	}
#else
    	/* handles value are composed of base value and index in service(s) array: */
    	GATT_ATTR_HANDLE_NBR_PUT(pGATTService->wHandleStart, pGATTService->wNbr);
    	pGATTService->wHandleStart++;   /* start with 0x*001 */
#endif
    }
    else    /* BQB mode */
    {
        uint32_t     service_index;
        uint16_t     max_handle = 0;
        PGATTService p_temp_service;
        PGATTServer  pServer = &pGATT->SrvCl.Server;

        /* find max handle used so far */
        for ( service_index = 0; service_index < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; service_index++ )
        {
            p_temp_service = &pServer->pService[service_index];
            if ( (p_temp_service != pGATTService) && p_temp_service->wUsed &&
                    ((p_temp_service->wHandleStart + p_temp_service->wAttribCnt - 1) > max_handle))
            {
                max_handle = p_temp_service->wHandleStart + p_temp_service->wAttribCnt - 1;
            }
        }

        if (max_handle == 0)
        {
            pGATTService->wHandleStart = max_handle + 1;
        }
        else
        {
            pGATTService->wHandleStart = (max_handle / 16 + 1) * 16;
        }

    }
#if F_BT_BREDR_SUPPORT   
	pGATTService->Sdp.wHandleNext = pGATTService->wHandleStart;//bredr
#endif
	GATT_TRACE_PRINTF_1(GATT_TRACE_MASK_TRACE, "GATT: gattServiceInit / wHandleStart=0x%x",
	                        pGATTService->wHandleStart);
}

/**
* @brief  init. service and create SDP record(s) if service(s) accessible over BR/EDR.
* 		it is assumed that pDeferredMsg contains the register req. message.
*
* @param pGATTService
* @param pDeferredMsg
*
* @return 
* @retval   > 0 : creation of SDP record was initiated or deferred.
* @retval  0   : no need to create SDP record.
* @retval  < 0 : error.
*/
int gattServicePrepare( PGATTService pGATTService,
                                               PGATTDeferredMsg pDeferredMsg )
{
	int                     iReturn = 0;


	gattServiceInit( pGATTService );

	if ( pDeferredMsg == NULL )
	{
		/* creation of SDP records is deferred until records of built-in   */
		/* services (GAP, GATT) have been created !!                       */
		iReturn = 1;
	}
	else
	{
#if F_BT_BREDR_SUPPORT
		osQueueIn( &pGATT->DeferredMsgQueueFree, pDeferredMsg );
		/* create SDP record for services that are accessible over BR/EDR */
		iReturn = gattSDPCreateDes( pGATTService, (PVOID)pDeferredMsg );
		if ( iReturn < 0 )
		{
		  assert(FALSE);
		}
#endif
	}

	return( iReturn );
}

/**
* @brief setup built-in GAP and GATT services.
* 
*
* @return 
* @retval > 0 : creation of SDP record was initiated.
* @retval  0   : no need to create SDP record.
* @retval < 0 : error.
*
*/
int gattGATTServiceBuiltinInit( void)
{
	PGATTService   pGATTService;
	int            iReturn = -1;

	/* allocate profile/service descriptor */
	if ( (pGATTService = gattServiceAlloc( )) != (PGATTService)0 )
	{
		pGATTService->pAttribFirst  = (PAttrib)gattBuiltinProfile;
		pGATTService->wAttribCnt    = sizeof(gattBuiltinProfile)/sizeof(TAttribAppl);

		gattServiceInit( pGATTService );
#if F_BT_BREDR_SUPPORT
		/* create SDP record for services that are accessible over BR/EDR */
		iReturn = gattSDPCreateDes( pGATTService, NULL );//bredr
		if ( iReturn < 0 )
		{
			/* error */
			assert(FALSE);
		}
#endif
	}

	return( iReturn );
}

/**
* @brief  get the handle range of an included service.
* 		it is assumed that pGATTService->pAttribFirst + iIndex references a
* 		valid primary or secondary service declaration!
* 
* @param pAttrib: attribute array 
* @param iIndex: index of attribute
* @param pStartHandle
* @param pEndHandle
*
* @return 
*
*/
int  gattIncludedServiceHandlesGet(
                  PAttrib       pAttrib,      
                  int           iIndex,       
                  uint16_t        * pStartHandle,
                  uint16_t        * pEndHandle )
{
	int           i, iReturn;
	PGATTService  pGATTService;

	iReturn = -2;

	/* find associated service/profile descriptor */
	//for ( i = 0; i < BT_GATT_SERVER_MAX_SERVICES_COUNT; i++)
	for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++)
	{
		pGATTService = &pGATT->pServer->pService[i];
		if ( pGATTService->wUsed && (pAttrib == pGATTService->pAttribFirst) )
		{
			iReturn = -1;
			break;
		}
	}

	if ( (iReturn > -2) && (iIndex < (pGATTService->wAttribCnt-1)) )
	{
		pAttrib = pGATTService->pAttribFirst + iIndex;
		/* must be service declaration */
		if ( gattAttribIsService( pAttrib ) )
		{
			*pStartHandle = pGATTService->wHandleStart + iIndex;

			pAttrib++;
			for ( i=iIndex+1; i<pGATTService->wAttribCnt; i++)
			{
				if ( gattAttribIsService( pAttrib ) )
				{
					/* found next service */
					break;
				}
				pAttrib++;
			}
			*pEndHandle = pGATTService->wHandleStart + (i-1);
			iReturn = 0;
		}
	}

	return( iReturn );
}

/**
* @brief		get pointer to attribute value
*
* @param	pAttrib
*
* @return
*
*/
uint8_t *  gattAttribValuePtrGet( PAttrib    pAttrib )
{
	uint8_t *  pValue = NULL;

	if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
	{
		/* value included in attribute (=> 16 bit UUID) */
		pValue = pAttrib->TypeValue.s.bValueIncl;
	}
	else if ( !(pAttrib->wFlags & ATTRIB_FLAG_VALUE_APPL) )
	{
		/* value not included in attribute but accessible */
		/* thru pValueContext */
		pValue = (uint8_t *)pAttrib->pValueContext;
	}
	else
	{
		/* value must be supplied by application */
	}

	return( pValue );
}

/**
* @brief		put End Group Handle into result list. if primary service was last in
* 			database return 0xFFFF.
*
* @param	Primary
* @param	SdpSearch
* @param	pDst
* @param	pGATTService
* @param	pGATTService
*
* @return
*
*/
void gattEndGroupHandlePut( BOOL Primary, BOOL SdpSearch, uint8_t * pDst,
                                       PGATTService pGATTService, int iIdx )
{
	uint16_t wEndGroupHandle = pGATTService->wHandleStart+iIdx-1;

#if (GATT_LAST_END_GROUP_HANDLE_FFFF)
	if ( Primary && !SdpSearch )
	{
		/* check if service is last in database */
		if ( (iIdx == pGATTService->wAttribCnt) &&   /* last attribute in descriptor */
		 //((pGATTService->wNbr >= (BT_GATT_SERVER_MAX_SERVICES_COUNT -1)) || /* last descriptor */
		 ((pGATTService->wNbr >= (otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count -1)) || /* last descriptor */
		  (!(pGATTService+1)->wUsed)                      /* last descriptor in use */
		 )
		)
		{
			wEndGroupHandle = 0xFFFF;
		}
	}
#endif

	LE_WORD2EXTRN( pDst, wEndGroupHandle );
}

/**
* @brief search for attributes (primary/secondary services, include definitions etc.)
* 
* @param SdpSearch: weather to call for SDP record setup
* @param pRClient
* @param LE : if discover over LE link
* @param wType: type of search
* @param Primary: primary or secondary service
* @param pwStartingHandle:if applic. has to supply value, the cur handle is returned
* @param iCmpValueLength:for search with UUID/value comparison
* @param pCmpValue
* @param piCnt : nbr. of attributes found
* @param pList: resulting list 
* @param piSize: input : (remaining) size in list
*				output: abs(*piSize) = size of list element
*				< 0: applic. has to supply value
* @param iListEntrySize:< 0: not yet defined, init. call
*
* @return 
*
*/
uint32_t  gattAttributesSearch( BOOL   SdpSearch,      
                      PGATTRClient pRClient,
                      BOOL   LE,            
                      uint16_t   wType,          
                      BOOL   Primary,       
                      LPWORD pwStartingHandle, 
                      uint16_t   wEndingHandle,
                      int    iCmpValueLength,
                      uint8_t * pCmpValue,
                      int   *piCnt,   
                      uint8_t * pList,
                      int   *piSize,
                      int    iListEntrySize 
                    )
{
	PGATTService pGATTService;
	PAttrib      pAttrib;
	uint16_t         wUUID16;
	int          i, iCnt;
	int          iReqListEntrySize;
	int          iSrvStartDescr;                  /* descr. to start with */
	int          iSrvStartIdx;                    /* start index          */
	int          iEndingHandleReached  = -1;
	BOOL         EndGroupHandlePending = FALSE;
	BOOL         Stop = FALSE;
	uint32_t        dwATTError = ATT_OK;

	iCnt   = 0;
	*piCnt = iCnt;

	iSrvStartDescr = GATT_ATTR_HANDLE_NBR_GET(*pwStartingHandle);
	iSrvStartIdx   = GATT_ATTR_HANDLE_IDX_GET(*pwStartingHandle);

	//if ( iSrvStartDescr >= BT_GATT_SERVER_MAX_SERVICES_COUNT )
	if ( iSrvStartDescr >= otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count )
	{
		return( ATT_ERR_ATTR_NOT_FOUND );
	}

	/* scan through all service descriptors */
	do
	{
		pGATTService = &pGATT->pServer->pService[iSrvStartDescr];
		if ( !pGATTService->wUsed || (pGATTService->wHandleStart > wEndingHandle) )
		{
			break;
		}
		pAttrib = pGATTService->pAttribFirst + iSrvStartIdx;

		for ( i = iSrvStartIdx; i < pGATTService->wAttribCnt; i++)
		{
		  	if ( iEndingHandleReached >= 0 )
			{
				iEndingHandleReached++;  /* count nbr. of attrib. following ending handle */
			}

			if ( !(pAttrib->wFlags & ATTRIB_FLAG_UUID_128BIT) )
			{
				/* most relevant declaration attributes have 16 bit UUIDs */
				wUUID16 = LE_EXTRN2WORD(pAttrib->TypeValue.s.bType);
			}
			else
			{
				wUUID16 = 0;
			}

		  	switch( wType )
			{
			default:
				Stop = TRUE;
				break;

			/*-------- primary (and secondary) service discovery --------*/
			case GATT_TYPE_DISCOVERY_PSRV_ALL:
			case GATT_TYPE_DISCOVERY_PSRV_UUID:
				if ( (wUUID16 == GATT_UUID_PRIMARY_SERVICE)   ||
				   (wUUID16 == GATT_UUID_SECONDARY_SERVICE)
				 )
				{
					/* check if output of End Group Handle to result list is outstanding */
					if ( EndGroupHandlePending )
					{
						gattEndGroupHandlePut( Primary, SdpSearch,
						                         pList+sizeof(uint16_t), pGATTService, i );
						EndGroupHandlePending = FALSE;
						*piSize -= iListEntrySize;
						pList   += iListEntrySize;
					}

					/* skip service declarations that don't match */
					if ( ((Primary && (wUUID16 == GATT_UUID_PRIMARY_SERVICE)) ||
					      (!Primary && (wUUID16 == GATT_UUID_SECONDARY_SERVICE))) &&
					      ((LE && (pAttrib->wFlags & ATTRIB_FLAG_LE)) ||
					       (!LE /*&& (pAttrib->wFlags & ATTRIB_FLAG_BR_EDR)*/)
					      )
					   )
					{
						uint8_t * pSrc;

						/* found service declaration */
						if ( wType == GATT_TYPE_DISCOVERY_PSRV_ALL )
						{
							if ( iListEntrySize < 0 )
							{
								/* first hit, list entry: 2 handles + value */
								iListEntrySize = 2*sizeof(uint16_t) + pAttrib->wValueLen;
							}
							else if ( iListEntrySize != 2*sizeof(uint16_t) + pAttrib->wValueLen )
							{
								/* change of value length (16 bit <-> 128 bit UUID ..),     */
								/* terminate search, client must issue new, updated request */
								Stop = TRUE;
								break;
							}

							/* try to create list entry: 2 handles + UUID must fit .. */
							if ( *piSize >= iListEntrySize )
							{
								iCnt++;

								LE_WORD2EXTRN( pList,
								    pGATTService->wHandleStart+i );  /* Attribute Handle */
								EndGroupHandlePending = TRUE;

								if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
								{
									pSrc = pAttrib->TypeValue.s.bValueIncl;
								}
								else
								{
									pSrc = (uint8_t *)pAttrib->pValueContext;
								}
								memcpy( pList+(2*sizeof(uint16_t)),        /* Attribute Value  */
								      pSrc, pAttrib->wValueLen );
							}
							else
							{
								Stop = TRUE;
							}
						}
						else /* GATT_TYPE_DISCOVERY_PSRV_UUID */
						{
							/* search for specific service/UUID */
							if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
							{
								pSrc = pAttrib->TypeValue.s.bValueIncl;
							}
							else
							{
								pSrc = (uint8_t *)pAttrib->pValueContext;
							}
							if ( memcmp(pCmpValue, pSrc, iCmpValueLength) != 0 )
							{
								pAttrib++;
								continue;
							}

							if ( iListEntrySize < 0 )
							{
								/* first hit, list entry: 2 handles */
								iListEntrySize = 2*sizeof(uint16_t);
							}

							/* try to create list entry: */
							if ( *piSize >= iListEntrySize )
							{
								iCnt++;

								LE_WORD2EXTRN( pList,
								    pGATTService->wHandleStart+i );  /* Attribute Handle */
								EndGroupHandlePending = TRUE;
							}
							else
							{
								Stop = TRUE;
							}
						}
					}
				}
			  	break;

			/*-------- relationship discovery --------*/
			case GATT_TYPE_DISCOVERY_RELATION:
			    if ( wUUID16 == GATT_UUID_INCLUDE )
			    {
					PAttrib  pInclService;

					/* locate included service and determine handle range */
					pInclService = pAttrib->pValueContext;
					pInclService += pAttrib->wValueLen;    /* index */

					if ( pInclService->wValueLen == UUID_128BIT_SIZE )
					{
						/* list entry: 3 handles */
						iReqListEntrySize = 3*sizeof(uint16_t);
					}
					else
					{
						/* list entry: 3 handles + 16 bit UUID */
						iReqListEntrySize = 3*sizeof(uint16_t) + UUID_16BIT_SIZE;
					}

					if ( iListEntrySize < 0 )
					{
						/* first hit */
						iListEntrySize = iReqListEntrySize;
					}
					else if ( iListEntrySize != iReqListEntrySize )
					{
						/* change of length, terminate search, client must issue */
						/* new, updated request                                  */
						Stop = TRUE;
						break;
					}

					/* try to create list entry: */
					if ( *piSize >= iListEntrySize )
					{
						uint16_t     wInclServiceAttHandle, wEndGroupHandle;

						/* determine included service handle range */
						if ( gattIncludedServiceHandlesGet(
						           pAttrib->pValueContext, /* attribute array    */
						           pAttrib->wValueLen,     /* index of attribute */
						           &wInclServiceAttHandle,
						           &wEndGroupHandle ) == 0
						   )
						{
							/* Attribute Handle */
							LE_WORD2EXTRN( pList, pGATTService->wHandleStart+i );
							pList += sizeof(uint16_t);
							/* Included Service Attribute Handle */
							LE_WORD2EXTRN( pList, wInclServiceAttHandle );
							pList += sizeof(uint16_t);
							/* End Group Handle */
							LE_WORD2EXTRN( pList, wEndGroupHandle );
							pList += sizeof(uint16_t);

							if ( iListEntrySize == (3*sizeof(uint16_t)+UUID_16BIT_SIZE) )
							{
								/* include 16 bit UUID in result */
								if ( pInclService->wFlags & ATTRIB_FLAG_VALUE_INCL )
								{
									*pList++ = pInclService->TypeValue.s.bValueIncl[0];
									*pList++ = pInclService->TypeValue.s.bValueIncl[1];
								}
								else
								{
									memcpy( pList, pInclService->pValueContext,
									                               UUID_16BIT_SIZE );
									pList += UUID_16BIT_SIZE;
								}
							}
							iCnt++;
							*piSize -= iListEntrySize;
						}
						else
						{
							assert(FALSE);
							Stop = TRUE;
						}
					}
					else
					{
						Stop = TRUE;
					}
			    }
			    break;

			/*-------- characteristic discovery --------*/
			case GATT_TYPE_DISCOVERY_CHAR_ALL:
			/*---------- get all UUIDs in search handle range  --------*/
			/* NOTE: wType=GATT_TYPE_DISCOVERY_CHAR_DESCR is mapped to */
			/*       wType=GATT_TYPE_DISCOVERY_GET_UUID !!!            */
			case GATT_TYPE_DISCOVERY_GET_UUID:
			    if ( (wUUID16 == GATT_UUID_CHARACTERISTIC) ||
			         (wType == GATT_TYPE_DISCOVERY_GET_UUID)
			       )
			    {
					PAttrib  pAttribSearch;
					int      iUUIDSize;

					/* determine list entry size */
					if ( wType == GATT_TYPE_DISCOVERY_CHAR_ALL )
					{
						/* char. value UUID is stored in next attribute, value */
						/* always follows declaration!:                        */
						pAttribSearch = pAttrib+1;

						/* list entry: decl. handle, properties, value handle + */
						/* 16/128 bit UUID                                      */
						iReqListEntrySize = 2*sizeof(uint16_t) + sizeof(uint8_t);
					}
					else
					{
						/* the current attribute is the one we are looking for .. */
						pAttribSearch = pAttrib;

						/* list entry: handle, 16/128 bit UUID */
						iReqListEntrySize = sizeof(uint16_t);
					}
					if ( pAttribSearch->wFlags & ATTRIB_FLAG_UUID_128BIT )
					{
						iUUIDSize          = UUID_128BIT_SIZE;
						iReqListEntrySize += UUID_128BIT_SIZE;
					}
					else
					{
						iUUIDSize          = UUID_16BIT_SIZE;
						iReqListEntrySize += UUID_16BIT_SIZE;
					}

					if ( iListEntrySize < 0 )
					{
						/* first hit */
						iListEntrySize = iReqListEntrySize;
					}
					else if ( iListEntrySize != iReqListEntrySize )
					{
						/* change of length, terminate search, client must issue */
						/* new, updated request                                  */
						Stop = TRUE;
						break;
					}

					/* try to create list entry: */
					if ( *piSize >= iListEntrySize )
					{
						/* Char. Declaration Handle resp. handle of current attrib. */
						LE_WORD2EXTRN( pList, pGATTService->wHandleStart+i );
						pList += sizeof(uint16_t);
						if ( wType == GATT_TYPE_DISCOVERY_CHAR_ALL )
						{
							/* Char. Properties */
							*pList = pAttrib->TypeValue.s.bValueIncl[0];
							pList += sizeof(uint8_t);
							/* Char. Value Handle */
							LE_WORD2EXTRN( pList, pGATTService->wHandleStart+i+1 );
							pList += sizeof(uint16_t);
						}
						/* Char. Value UUID resp. UUID of current attrib. */
						memcpy( pList, pAttribSearch->TypeValue.l.bType, iUUIDSize );
						pList += iUUIDSize;

						iCnt++;
						*piSize -= iListEntrySize;
					}
					else
					{
						Stop = TRUE;
					}
			    }
			    break;

			case GATT_TYPE_DISCOVERY_CMP_UUID:
			    /* search for specific value/UUID (pCmpValue points to UUID ..) */
			    if ( (((wUUID16 != 0) && (iCmpValueLength == UUID_16BIT_SIZE)) ||
			          ((wUUID16 == 0) && (iCmpValueLength == UUID_128BIT_SIZE))) &&
			         (memcmp(pCmpValue, pAttrib->TypeValue.l.bType,
			                                              iCmpValueLength) == 0)
			       )
			    {
					uint8_t *  pValue;
					int     iValueSize;

					dwATTError = gattAttribCheckPermissions( pRClient,
					                pGATTService->wHandleStart+i, pAttrib, FALSE );
					if ( dwATTError != ATT_OK )
					{
						Stop = TRUE;
					}
					else
					{
						/* determine list entry size: handle + value */
						pValue = gattAttribValuePtrGet( pAttrib );
						if ( (pValue != NULL) && (pAttrib->wFlags & ATTRIB_FLAG_ASCII_Z) )
						{
							iValueSize = strlen((const char *)pValue) + 1;
						}
						else
						{
							iValueSize = pAttrib->wValueLen;
						}
						iReqListEntrySize = sizeof(uint16_t) + iValueSize;

						if ( iListEntrySize < 0 )
						{
							/* first hit */
							iListEntrySize = iReqListEntrySize;
						}
						else if ( iListEntrySize != iReqListEntrySize )
						{
							/* change of length, terminate search, client must issue */
							/* new, updated request                                  */
							Stop = TRUE;
							break;
						}

						/* try to create list entry: */
						if ( *piSize > sizeof(uint16_t) )
						{
							iCnt++;

							/* attribute handle */
							LE_WORD2EXTRN( pList, pGATTService->wHandleStart+i );
							pList += sizeof(uint16_t);

							/* attribute value */
							if ( pValue == NULL )
							{
								/* value must be supplied by application */
								iListEntrySize    = -iListEntrySize;
								*pwStartingHandle = pGATTService->wHandleStart+i;
								Stop = TRUE;
								break;
							}
							else
							{
								if ( (int)(*piSize - sizeof(uint16_t)) < iValueSize )
								{
									iValueSize = (*piSize - sizeof(uint16_t));  /* reached MTU limit */
								}

								memcpy( pList, pValue, iValueSize );
								pList   += iValueSize;
								*piSize -= (sizeof(uint16_t) + iValueSize);
							}
						}
						else
						{
							Stop = TRUE;
						}
					}
				}
			    break;
			}  /* switch ... */

			if ( Stop )   /* exit for-loop .. */
			{
				break;
			}

			if ( iEndingHandleReached >= 1 )
			{
				Stop = TRUE;
				if ( EndGroupHandlePending )
				{
					/* next attribute following ending handle was not service declaration */
					iCnt--;
					EndGroupHandlePending = FALSE;
				}
				break;
			}

			/* check if ending handle is reached */
			if ( (iSrvStartDescr == GATT_ATTR_HANDLE_NBR_GET(wEndingHandle)) &&
			   (i == GATT_ATTR_HANDLE_IDX_GET(wEndingHandle)) )
			{
				iEndingHandleReached = 0;

				if ( (wType == GATT_TYPE_DISCOVERY_PSRV_ALL) ||
				     (wType == GATT_TYPE_DISCOVERY_PSRV_UUID) )
				{
				  /* if next attribute is a service declaration the service */
				  /* just found is completely included in the search range, */
				  /* if not ignore it ...                                   */
				}
				else
				{
					Stop = TRUE;
					break;
				}
			}

			pAttrib++;
		}  /* for ... */

		/* reached end of descriptor, service declarations don't span descr., */
		/* check if output of End Group Handle to result list is outstanding  */
		if ( EndGroupHandlePending )
		{
			gattEndGroupHandlePut( Primary, SdpSearch,
			                                 pList+sizeof(uint16_t), pGATTService, i );
			EndGroupHandlePending = FALSE;
			*piSize -= iListEntrySize;
			pList   += iListEntrySize;
		}

		/* proceed to next descriptor */
		//if ( !Stop && (iSrvStartDescr < (BT_GATT_SERVER_MAX_SERVICES_COUNT -1)) )
		if ( !Stop && (iSrvStartDescr < (otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count -1)) )
		{
			iSrvStartDescr++;
			iSrvStartIdx = 0;
		}
		else
		{
			break;  /* Stop = TRUE; */
		}
	}while( !Stop );

	*piCnt = iCnt;
	if (iCnt > 0 )
	{
		*piSize = iListEntrySize;
	}
	else
	{
		/* attribute not found or insufficient permissions */
		*piSize = 0;
		if ( dwATTError == ATT_OK )
		{
			dwATTError = ATT_ERR_ATTR_NOT_FOUND;
		}
	}

	return( dwATTError );
}

/**
* @brief		read attribute value
*
* @param	pRClient
* @param	wHandle
* @param	iOffset: offset in attribute value
* @param	pBuffer: output buffer
* @param	piSize
*				input : (remaining) size in buffer 
*             		output: size of attribute value if >= 0 
*             		otherw. applic. has to supply value  
*
* @return
*
*/
uint32_t  gattAttributeRead( PGATTRClient pRClient,
	                      uint16_t   wHandle,
	                      int    iOffset,   
	                      uint8_t * pBuffer,  
	                      int   *piSize    
                        )
{
	PAttrib      pAttrib;
	int          iSize;
	uint8_t *       pValue;
	uint16_t         wValueLen;
	uint32_t        dwATTError = ATT_ERR_INVALID_HANDLE;

	iSize     = *piSize;

	if ( (pAttrib = gattHandleToAttribute( wHandle )) != NULL )
	{
		dwATTError = gattAttribCheckPermissions( pRClient,
		                                                  wHandle, pAttrib, FALSE );
		if ( dwATTError == ATT_OK )
		{
			dwATTError = ATT_OK;

			if ( gattAttribIsCharClientCfg(pAttrib) )
			{
				/* special: read of client characteristic configuration attribute. */
				/* each client has its own instance of this attribute value!       */
				iSize = GATT_RCODE_CLIENT_CHAR_CFG;
			}
			else if ( gattAttribIsCharDecl(pAttrib) )
			{
				/* special: attribute value must be reconstructed: */
				int  i, iCnt;
				uint16_t wStartingHandle = wHandle;

				dwATTError = gattAttributesSearch( FALSE, pRClient,

				                      (pRClient->pL2CChannel->bdType & BLUEFACE_BDTYPE_LE_MASK),

				                      GATT_TYPE_DISCOVERY_CHAR_ALL, FALSE,
				                      &wStartingHandle, wHandle,
				                      0, NULL,
				                      &iCnt,
				                      pBuffer,
				                      &iSize /* size of value on return */,
				                      -1
				                                     );
				if ( dwATTError == ATT_OK )
				{
					/* search routine returns declaration handle in pBuffer, remove it: */
					iSize -= (sizeof(uint16_t)+iOffset);
					if ( iSize >= 0 )
					{
						for( i = 0; i < iSize; i++ )
						{
						  pBuffer[i] = pBuffer[i+sizeof(uint16_t)+iOffset];
						}
					}
					else
					{
						dwATTError = ATT_ERR_INVALID_OFFSET;
					}
				}
			}
			else
			{
				/*--- "normal" attributes .. ---*/
				wValueLen = pAttrib->wValueLen;
				if ( wValueLen == 0 )
				{
					/* attribute value has variable size */
					if ( !(pAttrib->wFlags & ATTRIB_FLAG_VALUE_APPL) &&
					   (pAttrib->wFlags & ATTRIB_FLAG_ASCII_Z) )
					{
						/* attribute value is ASCII-Z string and is directly accessible */
						if ( pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL )
						{
							/* value included in attribute */
							wValueLen = strlen( (const char *)pAttrib->TypeValue.s.bValueIncl );
						}
						else
						{
							wValueLen = strlen( pAttrib->pValueContext );
						}
						wValueLen++;
					}
					else
					{
						/* must be supplied by application */
						iSize = GATT_RCODE_APPLIC_VALUE;
					}
				}

				if ( iSize != GATT_RCODE_APPLIC_VALUE )
				{
					if ( iOffset >= wValueLen )
					{
						iSize = 0;
						if ( iOffset > wValueLen )
						{
							dwATTError = ATT_ERR_INVALID_OFFSET;
						}
					}
					else
					{
						pValue = gattAttribValuePtrGet( pAttrib );
						if ( pValue == NULL )
						{
							/* value must be supplied by application */
							iSize = GATT_RCODE_APPLIC_VALUE;
						}
						else
						{
							/* value is directly accessible, copy it .. */
							if ( iSize > (wValueLen - iOffset) )
							{
								iSize = wValueLen - iOffset;
							}
							if ( iSize > 0 )
							{
								memcpy( pBuffer, pValue+iOffset, iSize );
							}
						}
					}
				}
			}
		}
	}

	*piSize = iSize;
	return( dwATTError );
}

/**
* @brief		write attribute value
*
* @param	pRClient: 
* @param	wHandle
* @param	pAttValue 
* @param	piSize
*				input : size of attribute value
*				output: nbr. of bytes written if >= 0
*				otherw. applic. has to write value
* @param	Cmd
*
* @return
*
*/
uint32_t  gattAttributeWrite( PGATTRClient pRClient,
                             uint16_t   wHandle,
                             uint8_t * pAttValue,
                             int    *piSize,
                             BOOL   Cmd
                         )
{
	PAttrib      pAttrib;
	int          iSize;
	uint8_t *       pValue;
	uint32_t        dwATTError = ATT_ERR_INVALID_HANDLE;

	iSize     = *piSize;

	if ( (pAttrib = gattHandleToAttribute( wHandle )) != NULL )
	{
		dwATTError = gattAttribCheckPermissions( pRClient,
		                                                  wHandle, pAttrib, TRUE );
		if ( dwATTError == ATT_OK )
		{
			if ( gattAttribIsCharClientCfg(pAttrib) )
			{
				/* special: write to client characteristic configuration attribute. */
				/* each client has its own instance of this attribute value!        */
				iSize = GATT_RCODE_CLIENT_CHAR_CFG;
			}
			else
			{
				/* pAttrib cannot point to first attribute (service decl.) because */
				/* permissions allow writing => -1 is OK here:                     */
				if ( gattAttribIsCharDecl( pAttrib-1 ) )
				{
					/* special: write to characteristic value attribute, */
					/* cross-check with properties:                      */
					PAttrib      pAttribCharDef = pAttrib-1;

					if ( (Cmd && (pAttribCharDef->TypeValue.s.bValueIncl[0] &
					                            GATT_CHAR_PROP_WRITE_NO_RSP) == 0) ||
					   (!Cmd && (pAttribCharDef->TypeValue.s.bValueIncl[0] &
					                            GATT_CHAR_PROP_WRITE) == 0)
					 )
					{
						/* XXXXMJMJ: error code like "write type mismatch" is missing ..: */
						dwATTError = ATT_ERR_WRITE_NOT_PERMITTED;
					}
				}

				if ( dwATTError == ATT_OK )
				{
					pValue = gattAttribValuePtrGet( pAttrib );
					if ( (pAttrib->wValueLen == 0) || /* attribute value has variable size */
					   (pValue == NULL)
					 )
					{
						/* value must be written by application */
						iSize = GATT_RCODE_APPLIC_VALUE;
					}
					else
					{
						/* value is directly accessible => fixed attribute value length */
						/* assumed, copy it ..                                          */
						if ( iSize > pAttrib->wValueLen  )
						{
							iSize      = 0;
							dwATTError = ATT_ERR_INVALID_VALUE_SIZE;
						}
						else if ( iSize > 0 )
						{
							memcpy( pValue, pAttValue, iSize );
						}
					}
				}
			}
		}
	}

	*piSize = iSize;
	return( dwATTError );
}

uint32_t  gattCheckPrepareWrite( PGATTRClient pRClient, uint16_t wHandle)
{
    PAttrib      pAttrib;
    uint32_t        dwATTError = ATT_ERR_INVALID_HANDLE;

    if ( (pAttrib = gattHandleToAttribute( wHandle )) != NULL )
    {
        dwATTError = gattAttribCheckPermissions( pRClient,
                     wHandle, pAttrib, TRUE );
        if ( dwATTError == ATT_OK )
        {
            if ( gattAttribIsCharClientCfg(pAttrib) )
            {
                dwATTError = ATT_ERR_INVALID_VALUE;
            }
            else
            {
                /* pAttrib cannot point to first attribute (service decl.) because */
                /* permissions allow writing => -1 is OK here:                  */
                if ( gattAttribIsCharDecl( pAttrib - 1 ) )
                {
                    /* special: write to characteristic value attribute, */
                    /* cross-check with properties:                   */
                    PAttrib   pAttribCharDef = pAttrib - 1;

                    if ( (pAttribCharDef->TypeValue.s.bValueIncl[0] &
                            GATT_CHAR_PROP_WRITE) == 0)
                    {
                        /* XXXXMJMJ: error code like "write type mismatch" is missing ..: */
                        dwATTError = ATT_ERR_WRITE_NOT_PERMITTED;
                    }
                }
            }
        }
    }
    else
    {
        dwATTError = ATT_ERR_INVALID_HANDLE;
    }
    return ( dwATTError );
}

/**
* @brief get characteristic properties
* 
* @param pAttrib
*
*/
int gattCharPropertiesGet( PAttrib pAttrib )
{
	int iProperties;

	if ( (pAttrib->wFlags & ATTRIB_FLAG_VALUE_INCL) &&
	   (LE_EXTRN2WORD(pAttrib->TypeValue.s.bType) == GATT_UUID_CHARACTERISTIC)
	 )
	{
		iProperties = pAttrib->TypeValue.s.bValueIncl[0];
	}
	else
	{
		/* error, illegal coding of <<Characteristic>> */
		iProperties = -1;
	}

	return( iProperties );
}

/**
* @brief check if ATT PDU handling has to be deferred if channel is not yet ready.
* 
* @param pData
*
*/
BOOL gattDeferMsgIfChannelNotReady( uint8_t * pData )
{
	LPATTPDU pATTPDU = (LPATTPDU)pData;

	return( !((pATTPDU->Opcode == ATT_WRITE_COMMAND) ||
	        ((pATTPDU->Opcode & ATT_OPCODE_MASK) == ATT_HANDLE_VALUE_NOTIFICATION) ||
	        ((pATTPDU->Opcode & ATT_OPCODE_MASK) == ATT_EXCHANGE_MTU_REQUEST)
	       )
	    );
}

/**
* @brief save/defer LL msg buffer
* 
* @param pL2CChannel
*
*/
void gattDeferredMsgLLSave( PGATTL2CChannel pL2CChannel, MESSAGE_P pMsg)
{
	pL2CChannel->DeferredMsgLL.pBuffer = pMsg->MData.DataCB.BufferAddress;
	pL2CChannel->DeferredMsgLL.wLength = pMsg->MData.DataCB.Length;
	pL2CChannel->DeferredMsgLL.wOffset = pMsg->MData.DataCB.Offset;
	pL2CChannel->DeferredMsgLL.wFlags  = pMsg->MData.DataCB.Flag;
}

/**
* @brief  free deferred LL msg buffer
*
* @param  pL2CChannel
* @param  ReleaseBuffer
*
* @return  
*
*/

void gattDeferredMsgLLFree( PGATTL2CChannel pL2CChannel,
                                                            BOOL ReleaseBuffer)
{
	if ( pL2CChannel->DeferredMsgLL.pBuffer != NULL )
	{
		if ( ReleaseBuffer && (pL2CChannel->DeferredMsgLL.wFlags & DATA_CB_RELEASE) )
		{
	  		osBufferRelease( pL2CChannel->DeferredMsgLL.pBuffer );
		}
		pL2CChannel->DeferredMsgLL.pBuffer = NULL;
	}
}

/**
* @brief  process deferred LL msg
*
* @param  pL2CChannel
*
* @return  
*
*/
void gattDeferredMsgLLProcess( PGATTL2CChannel pL2CChannel )
{
	uint8_t * pBuffer;
	uint16_t   wLength;
	uint16_t   wOffset;

	if ( pL2CChannel->DeferredMsgLL.pBuffer != NULL )
	{
		BOOL ReleaseBuffer;

		pBuffer = pL2CChannel->DeferredMsgLL.pBuffer;
		wLength = pL2CChannel->DeferredMsgLL.wLength;
		wOffset = pL2CChannel->DeferredMsgLL.wOffset;
		
		pGATT->pBuffer = pL2CChannel->DeferredMsgLL.pBuffer;
		ReleaseBuffer  = gattLLDataReceived(pL2CChannel, pBuffer, wLength, wOffset );
		gattDeferredMsgLLFree( pL2CChannel, ReleaseBuffer );
	}
}



/**
* @brief   map causes received from applic. to ATT error codes
* 
* @param wCause:
*
* @return
*
*/

uint8_t gattCauseToATTError( uint16_t wCause )
{
	uint8_t   bATTError;
	uint16_t   wLocation = wCause & 0xFF00;

	/* map all causes to ATT_ERR_UNLIKELY with a few exceptions */
	bATTError = ATT_ERR_UNLIKELY;

	if ( wLocation == 0 )
	{
		wLocation = ATT_ERR;
	}

	if ( wLocation == ATT_ERR )
	{
		bATTError = (uint8_t)wCause;

		if ( (bATTError > ATT_ERR_INSUFFICIENT_RESOURCES) &&
		     (bATTError < ATT_ERR_MIN_APPLIC_CODE)
		   )
		{
			bATTError = ATT_ERR_UNLIKELY;
		}
	}

	return( bATTError );
}

/**
* @brief  init gatt data
* 
*
* @return  init result
*
*/
void gattInitData()
{
	int               i;
	PGATTServer       pServer = &pGATT->SrvCl.Server;

	pGATT->pServer = pServer;
#if F_BT_BREDR_SUPPORT
	{
		PGATTDeferredMsg  pDeferredMsg;

		/* init. q of free deferred msg descriptors */
		pDeferredMsg = &pGATT->DeferredMsg[0];
		for ( i = 0; i < GATT_MAX_DEFERRED_MSGS; i++ )
		{
			osQueueIn( &pGATT->DeferredMsgQueueFree, pDeferredMsg );
			pDeferredMsg++;
		}
	}
#endif

    pGATT->pL2CChannelDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTL2CChannel)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pGATT->pL2CChannelDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TGATTL2CChannel)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    pGATT->SrvCl.pClientDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTClient)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pGATT->SrvCl.pClientDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TGATTClient)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);

    pGATT->SrvCl.Server.pRClientDon = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTRClient)*otp_str_data.gEfuse_UpperStack_s.num_link_don);
    pGATT->SrvCl.Server.pRClientDoff = osMemoryAllocate(RAM_TYPE_DATA_OFF, sizeof(TGATTRClient)*otp_str_data.gEfuse_UpperStack_s.num_link_doff);


    pGATT->SrvCl.Server.pService = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTService)*otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count);
    pServer->pCCCBits = osMemoryAllocate(RAM_TYPE_DATA_ON, sizeof(TGATTCCCBitsTable)*otp_str_data.gEfuse_UpperStack_s.gattServerCCCbitsCount);

     /* L2CAP channel descriptors */
    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_don; i++)
    {
        pGATT->pL2CChannelDon[i].wNbr = i ;
    }

    for (i = 0; i < otp_str_data.gEfuse_UpperStack_s.num_link_doff; i++)
    {
        pGATT->pL2CChannelDoff[i].wNbr = i + 0x10;
    }

    /* link L2CAP channels to remote and/or local client descriptors */
    gattL2CChannelsLinkToClientDescr( );

	pGATT->wRxOffset = DATACB_INIT_OFFSET;

    memset( pServer->pService, 0, otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count * sizeof(TGATTService));
    for ( i = 0; i < otp_str_data.gEfuse_UpperStack_s.gatt_max_service_count; i++ )
    {
		pServer->pService[i].wNbr = i;
	}
}

