/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_ndcs.c
* @brief     GATT built-in NDCS (Next DST (Daylight Saving Time) Change Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>

#ifndef __GATTSVC_NDCS_H
#include <gattsvc_ndcs.h>
#endif

/**
 * @brief  service definition.
 *
 *	 Next DST (Daylight Saving Time) Change Service 
 */
const TAttribAppl gattSvcNDCS[] =
{
	/** Primary Service */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /**< wFlags     */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_NEXT_DST_CHANGE),     /**< service UUID */
			HI_WORD(GATT_UUID_NEXT_DST_CHANGE)
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
	/** Time with DST value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST),
			HI_WORD(GATT_UUID_CHAR_NDCS_TIME_WITH_DST)
		},
		8,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	}
};

const int gattSvcNDCSSize        = sizeof(gattSvcNDCS);
const int gattSvcNDCSNbrOfAttrib = sizeof(gattSvcNDCS) / sizeof(TAttribAppl);

