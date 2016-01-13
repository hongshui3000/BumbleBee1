/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       gatt.h
* @brief     gatt 
* @details   
*
* @author   	gordon
* @date      	2015-07-09
* @version	v0.1
*/

#ifndef __GATT_H
#define __GATT_H


#ifdef __cplusplus
extern "C" {
#endif


/** @brief GATT service and characteristic definition codes*/
#define GATT_UUID_PRIMARY_SERVICE       0x2800
#define GATT_UUID_SECONDARY_SERVICE     0x2801
#define GATT_UUID_INCLUDE               0x2802
#define GATT_UUID_CHARACTERISTIC        0x2803

/** @brief GATT characteristic descriptors */
#define GATT_UUID_CHAR_EXTENDED_PROP    0x2900
#define GATT_UUID_CHAR_USER_DESCR       0x2901
#define GATT_UUID_CHAR_CLIENT_CONFIG    0x2902
#define GATT_UUID_CHAR_SERVER_CONFIG    0x2903
#define GATT_UUID_CHAR_FORMAT           0x2904
#define GATT_UUID_CHAR_AGGR_FORMAT      0x2905
#define GATT_UUID_CHAR_VALID_RANGE      0x2906
#define GATT_UUID_CHAR_EXTERNAL_REPORT_REFERENCE 0x2907
#define GATT_UUID_CHAR_REPORT_REFERENCE 0x2908
#define GATT_UUID_CHAR_SENSING_CONFIGURATION 0x290B
#define GATT_UUID_CHAR_SENSING_MEASUREMENT 0x290C
#define GATT_UUID_CHAR_SENSING_TRIGGER_SETTING 0x290D

/** @brief GATT service/profile characteristics */
#define GATT_UUID_CHAR_DEVICE_NAME          0x2A00
#define GATT_UUID_CHAR_APPEARANCE           0x2A01
#define GATT_UUID_CHAR_PER_PRIV_FLAG        0x2A02
#define GATT_UUID_CHAR_RECONN_ADDRESS       0x2A03
#define GATT_UUID_CHAR_PER_PREF_CONN_PARAM  0x2A04
#define GATT_UUID_CHAR_SERVICE_CHANGED      0x2A05


/* GATT attribute read/write permissions, encryption key size */
/** @brief TAttribAppl.wPermissions bits 0..15:                       */ 
#define GATT_PERM_NONE                   0x00
#define GATT_PERM_ALL                    0x01  /**< bits 0..1 (rd), 4..5 (wr), 8..9 (notif/ind) */
#define GATT_PERM_AUTHEN_REQ             0x02
#define GATT_PERM_AUTHEN_MITM_REQ        0x03
#define GATT_PERM_AUTHOR_REQ             0x04  /**< bits 2 (rd), 6 (wr), 10 (notif/ind) */
#define GATT_PERM_RESERVED               0x08  /**< bits 3 (rd), 7 (wr), 11 (notif/ind) */

/** @brief read (bits 0..3) */
#define GATT_PERM_READ                   GATT_PERM_ALL
#define GATT_PERM_READ_AUTHEN_REQ        GATT_PERM_AUTHEN_REQ
#define GATT_PERM_READ_AUTHEN_MITM_REQ   GATT_PERM_AUTHEN_MITM_REQ
#define GATT_PERM_READ_AUTHOR_REQ        GATT_PERM_AUTHOR_REQ

#define GATT_PERM_READ_AUTHEN_GET(x)     (x & 0x03)
#define GATT_PERM_READ_AUTHOR_GET(x)     (x & 0x04)

/** @brief write (bits 4..7) */
#define GATT_PERM_WRITE                  (GATT_PERM_ALL << 4)
#define GATT_PERM_WRITE_AUTHEN_REQ       (GATT_PERM_AUTHEN_REQ << 4)
#define GATT_PERM_WRITE_AUTHEN_MITM_REQ  (GATT_PERM_AUTHEN_MITM_REQ << 4)
#define GATT_PERM_WRITE_AUTHOR_REQ       (GATT_PERM_AUTHOR_REQ << 4)

#define GATT_PERM_WRITE_AUTHEN_GET(x)    ((x >> 4) & 0x03)
#define GATT_PERM_WRITE_AUTHOR_GET(x)    ((x >> 4) & 0x04)

/** @brief notification/indication (bits 8..11) */
#define GATT_PERM_NOTIF_IND                  (GATT_PERM_ALL << 8)
#define GATT_PERM_NOTIF_IND_AUTHEN_REQ       (GATT_PERM_AUTHEN_REQ << 8)
#define GATT_PERM_NOTIF_IND_AUTHEN_MITM_REQ  (GATT_PERM_AUTHEN_MITM_REQ << 8)
#define GATT_PERM_NOTIF_IND_AUTHOR_REQ       (GATT_PERM_AUTHOR_REQ << 8)

#define GATT_PERM_NOTIF_IND_AUTHEN_GET(x)    ((x >> 8) & 0x03)
#define GATT_PERM_NOTIF_IND_AUTHOR_GET(x)    ((x >> 8) & 0x04)


/** @brief key size - 1 (bits 12..15) */
#define GATT_PERM_KEYSIZE(size)          ((size-1) << 12)
#define GATT_PERM_KEYSIZE_GET(x, size)   {                             \
	                                         size = ((x >> 12) & 0x0F);  \
                                           if ( size > 0 )             \
                                             size++;                   \
                                         }



/** @brief GATT characteristic properties */
#define GATT_CHAR_PROP_BROADCAST          0x01
#define GATT_CHAR_PROP_READ               0x02
#define GATT_CHAR_PROP_WRITE_NO_RSP       0x04
#define GATT_CHAR_PROP_WRITE              0x08
#define GATT_CHAR_PROP_NOTIFY             0x10
#define GATT_CHAR_PROP_INDICATE           0x20
#define GATT_CHAR_PROP_WRITE_AUTH_SIGNED  0x40
#define GATT_CHAR_PROP_EXT_PROP           0x80

/** @brief GATT client characteristic configuration bit field */
#define GATT_CLIENT_CHAR_CONFIG_DEFAULT   0x0000
#define GATT_CLIENT_CHAR_CONFIG_NOTIFY    0x0001
#define GATT_CLIENT_CHAR_CONFIG_INDICATE  0x0002

/** @brief GATT server characteristic configuration bit field */
#define GATT_SERVER_CHAR_CONFIG_BROADCAST 0x0001



#define UUID_16BIT_SIZE                 2
#define UUID_128BIT_SIZE                16


#define HI_WORD(x)  ((uint8_t)((x & 0xFF00) >> 8))
#define LO_WORD(x)  ((uint8_t)(x))


/*---------------------------------------------------------------------------
 * GATT server attribute descriptor
 *--------------------------------------------------------------------------*/

/*--- flags ---*/
#define ATTRIB_FLAG_VOID          0x0000
#define ATTRIB_FLAG_UUID_128BIT   0x0001
/** @brief attribute value is included following 16 bit UUID: */
#define ATTRIB_FLAG_VALUE_INCL    0x0002
/** @brief application has to supply/write value: */
#define ATTRIB_FLAG_VALUE_APPL    0x0004
/** @brief attribute value is ASCII_Z string: */
#define ATTRIB_FLAG_ASCII_Z       0x0008
/** @brief application has to be informed about CCCD value (changes): */
#define ATTRIB_FLAG_CCCD_APPL     0x0010
/* evaluated only for primary service declaration attributes, if   */
/** @brief set to 1 the service is available using GATT over BR/EDR links: */
#define ATTRIB_FLAG_BR_EDR        0x0400
/* evaluated only for primary service declaration attributes, if   */
/** @brief set to 1 the service is available using GATT over LE links:     */
#define ATTRIB_FLAG_LE            0x0800


typedef struct _AttribAppl
{
    uint16_t    wFlags;
    uint8_t     bTypeValue[2+14];   /**< 16 bit UUID + included value or 128 bit UUID */
    uint16_t    wValueLen;          /**< length of value              */
    void        *pValueContext;     /**< ptr to value if not included or applic. supplied */
                                    /**< context if flag ATTRIB_FLAG_VALUE_APPL is set    */
    uint16_t    wPermissions;       /**< read/write permissions, encryption key size ..   */
} TAttribAppl, * PAttribAppl;

#ifdef __cplusplus
}
#endif

#endif /**< __GATT_H */

