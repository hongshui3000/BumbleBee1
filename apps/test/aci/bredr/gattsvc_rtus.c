/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_rtus.c
* @brief     GATT built-in RTUS (Reference Time Update Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>

#ifndef __GATTSVC_RTUS_H
#include <gattsvc_rtus.h>
#endif

/**
 * @brief  service definition.
 *
 *	 Reference Time Update Service 
 */
const TAttribAppl gattSvcRTUS[] =
{
	/** Primary Service*/
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  /**< wFlags     */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_REFERENCE_TIME_UPDATE), /**< service UUID */
			HI_WORD(GATT_UUID_REFERENCE_TIME_UPDATE)
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
			GATT_CHAR_PROP_WRITE_NO_RSP             /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** Time Update Control Point value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT),
			HI_WORD(GATT_UUID_CHAR_RTUS_CONTROL_POINT)
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_WRITE                             /**< wPermissions */
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
	/** Time Update State value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_RTUS_STATE),
			HI_WORD(GATT_UUID_CHAR_RTUS_STATE)
		},
		2,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /* wPermissions */
	}
};

const int gattSvcRTUSSize        = sizeof(gattSvcRTUS);
const int gattSvcRTUSNbrOfAttrib = sizeof(gattSvcRTUS) / sizeof(TAttribAppl);

