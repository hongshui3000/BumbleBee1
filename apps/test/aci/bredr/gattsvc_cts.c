/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_cts.c
* @brief     GATT built-in GLS (Current Time Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>

#ifndef __GATTSVC_CTS_H
#include <gattsvc_cts.h>
#endif

/** CTS Current Time Service */
#define GATT_UUID_CURRENT_TIME                 0x1805

#define GATT_UUID_CHAR_CTS_CURRENT_TIME        0x2A2B
#define GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO     0x2A0F
#define GATT_UUID_CHAR_CTS_REF_TIME_INFO       0x2A14
/**
 * @brief  service definition.
 *
 *	Current Time Service
 */
const TAttribAppl gattSvcCTS[] =
{
	/** Primary Service */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  	/**< wFlags     */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_CURRENT_TIME),          	/**< service UUID */
			HI_WORD(GATT_UUID_CURRENT_TIME)
		},
		UUID_16BIT_SIZE,                            	/**< bValueLen     */
		NULL,                                       	/**< pValueContext */
		GATT_PERM_READ                              	/**< wPermissions  */
	},

	/**Characteristic>>*/
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			(GATT_CHAR_PROP_READ |                    	/**< characteristic properties */
			        GATT_CHAR_PROP_NOTIFY)
		/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Current Time value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME),
			HI_WORD(GATT_UUID_CHAR_CTS_CURRENT_TIME)
		},
		10,                                         	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
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
	},

	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                       	/**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Local Time Information value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO),
			HI_WORD(GATT_UUID_CHAR_CTS_LOCAL_TIME_INFO)
		},
		2,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},

	/** Characteristic*/
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_READ                       	/**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Reference Time Information value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO),
			HI_WORD(GATT_UUID_CHAR_CTS_REF_TIME_INFO)
		},
		4,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	}
};

const int gattSvcCTSSize        = sizeof(gattSvcCTS);
const int gattSvcCTSNbrOfAttrib = sizeof(gattSvcCTS) / sizeof(TAttribAppl);

