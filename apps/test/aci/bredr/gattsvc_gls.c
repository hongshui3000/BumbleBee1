/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       Gattsvc_gls.c
* @brief     GATT built-in GLS (Glucose Service)
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

//#include <flags.h>
#include <rtl_types.h>

#ifndef __GATTSVC_GLS_H
#include <gattsvc_gls.h>
#endif


/**
 * @brief  service definition.
 *
 *	Glucose Servic
 */
const TAttribAppl gattSvcGLS[] =
{
	/** Primary Service>>, .. */
	{
		(ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_LE),  	/**< wFlags     */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_PRIMARY_SERVICE),
			HI_WORD(GATT_UUID_PRIMARY_SERVICE),
			LO_WORD(GATT_UUID_GLUCOSE),               	/**< service UUID */
			HI_WORD(GATT_UUID_GLUCOSE)
		},
		UUID_16BIT_SIZE,                            /**< bValueLen     */
		NULL,                                       /**< pValueContext */
		GATT_PERM_READ                              /**< wPermissions  */
	},

	/** Characteristic*/
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_NOTIFY,                    	/**< characteristic properties */
			//XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Glucose Measurement characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT),
			HI_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT)
		},
		0,                                          	/**< variable size */
		(void*)NULL,
		GATT_PERM_NONE                              	/**< wPermissions */
	},
	/** client characteristic configuration */
	{
		(ATTRIB_FLAG_VALUE_INCL |                   	/**< wFlags */
		   ATTRIB_FLAG_CCCD_APPL),
		{                                           	/* bTypeValue */
			LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
			HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
			/* NOTE: this value has an instantiation for each client, a write to */
			/* this attribute does not modify this default value:                */
			LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), 	/**<client char. config. bit field */
			HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
		},
		2,                                          	/**< bValueLen */
		NULL,
		(GATT_PERM_READ | GATT_PERM_WRITE)          	/**< wPermissions */
	},

	/** Characteristic  */
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			GATT_CHAR_PROP_NOTIFY,                    	/**< characteristic properties */
			//XXXXMJMJ GATT_CHAR_PROP_INDICATE,                  /**< characteristic properties */
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Glucose Measurement Context characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT),
			HI_WORD(GATT_UUID_CHAR_GLS_MEASUREMENT_CTXT)
		},
		0,                                          	/**< variable size */
		(void*)NULL,
		GATT_PERM_NONE                              	/**< wPermissions */
	},
	/** client characteristic configuration */
	{
		(ATTRIB_FLAG_VALUE_INCL |                   	/**< wFlags */
		   ATTRIB_FLAG_CCCD_APPL),
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
		(GATT_PERM_READ | GATT_PERM_WRITE)          	/**<wPermissions */
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
	/** Glucose Features characteristic value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_GLS_FEATURES),
			HI_WORD(GATT_UUID_CHAR_GLS_FEATURES),
		},
		2,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Characteristic */
	{
		ATTRIB_FLAG_VALUE_INCL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHARACTERISTIC),
			HI_WORD(GATT_UUID_CHARACTERISTIC),
			(GATT_CHAR_PROP_WRITE |                  	/**< characteristic properties */
			             GATT_CHAR_PROP_INDICATE)	
			/* characteristic UUID not needed here, is UUID of next attrib. */
		},
		1,                                          	/**< bValueLen */
		NULL,
		GATT_PERM_READ                              	/**< wPermissions */
	},
	/** Glucose Record Access Control Point value */
	{
		ATTRIB_FLAG_VALUE_APPL,                     	/**< wFlags */
		{                                           	/**< bTypeValue */
			LO_WORD(GATT_UUID_CHAR_GLS_RACP),
			HI_WORD(GATT_UUID_CHAR_GLS_RACP)
		},
		0,                                          	/**< bValueLen, 0 : variable length */
		NULL,
		GATT_PERM_WRITE_AUTHEN_REQ                  	/**< wPermissions */
	},
	/** client characteristic configuration */
	{
		(ATTRIB_FLAG_VALUE_INCL |                   	/**< wFlags */
		   ATTRIB_FLAG_CCCD_APPL),
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

const int gattSvcGLSSize        = sizeof(gattSvcGLS);
const int gattSvcGLSNbrOfAttrib = sizeof(gattSvcGLS) / sizeof(TAttribAppl);

