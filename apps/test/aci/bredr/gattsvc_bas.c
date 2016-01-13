/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_bas.c
* @brief     GATT built-in BAS (Battery Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>
#ifndef __GATTSVC_BAS_H
#include <gattsvc_bas.h>
#endif


/**
 * @brief  service definition.
 *
 *	Battery Service
 */
const TAttribAppl gattSvcBAS[] =
{
	/** primary Service */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE | ATTRIB_FLAG_BR_EDR),   /**< wFlags  */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_BATTERY),               	/**< service UUID */
			HI_WORD(GATT_UUID_BATTERY)
		},
		UUID_16BIT_SIZE,                            	/**< bValueLen     */
		NULL,                                       	/**< ValueContext */
		GATT_PERM_READ                              	/**< wPermissions  */
	},

	/**  Characteristic  */
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			(GATT_CHAR_PROP_READ |                    	/**< characteristic properties */
			        GATT_CHAR_PROP_NOTIFY)
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Battery Level value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_BAS_LEVEL),
			HI_WORD(GATT_UUID_CHAR_BAS_LEVEL)
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                             		/**< wPermissions */
	},
	/** client characteristic configuration */
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
			HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
			/* NOTE: this value has an instantiation for each client, a write to */
			/* this attribute does not modify this default value:                */
			LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), 	/**< client char. config. bit field */
			HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
		},
		2,                                          	/**< bValueLen */
		NULL,
		(GATT_PERM_READ | GATT_PERM_WRITE)          	/**< wPermissions */
	}
};

const int gattSvcBASSize        = sizeof(gattSvcBAS);
const int gattSvcBASNbrOfAttrib = sizeof(gattSvcBAS) / sizeof(TAttribAppl);

