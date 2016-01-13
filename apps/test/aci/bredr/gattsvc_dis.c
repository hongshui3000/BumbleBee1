/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gattsvc_dis.c
* @brief     device information service
* @details   
*
* @author  	gordon
* @date      	2015-06-30
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>

#ifndef __GATTSVC_DIS_H
#include <gattsvc_dis.h>
#endif


/**
* @brief		device information service
*
*
*/
const TAttribAppl gattSvcDIS[] =
{
	/** Primary Service */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE | ATTRIB_FLAG_BR_EDR),   /**< wFlags     */
		{                                          			 	/**<bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_DEVICE_INFORMATION_SERVICE),  	/**< service UUID */
			HI_WORD(GATT_UUID_DEVICE_INFORMATION_SERVICE)
		},
		UUID_16BIT_SIZE,                            /**< bValueLen     */
		NULL,                                       /**< pValueContext */
		GATT_PERM_READ                              /**<wPermissions  */
	},

	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                       /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** System ID String characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_SYSTEM_ID),
			HI_WORD(GATT_UUID_CHAR_SYSTEM_ID)
		},
		0,                                          /**< variable size */
		(void*)NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},

	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                       /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** Manufacturer Name String characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_MANUFACTURER_NAME),
			HI_WORD(GATT_UUID_CHAR_MANUFACTURER_NAME)
		},
		0,                                          /**< variable size */
		(void*)NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},

	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                       /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          /**< bValueLen */
		NULL,
		GATT_PERM_READ                              /**< wPermissions */
	},
	/** Model Number characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     /**< wFlags */
		{                                           /**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_MODEL_NUMBER),
			HI_WORD(GATT_UUID_CHAR_MODEL_NUMBER)
		},
		0,                                          /**< variable size */
		(void*)NULL,
		GATT_PERM_READ                              /**< wPermissions */
	}
};

const int gattSvcDISSize        = sizeof(gattSvcDIS);
const int gattSvcDISNbrOfAttrib = sizeof(gattSvcDIS) / sizeof(TAttribAppl);

