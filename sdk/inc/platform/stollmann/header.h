/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file       header.h
* @brief     Module Header Definitions
* @details   
*
* @author  	gordon
* @date      	2015-07-13
* @version	v0.1
*/
#if !defined(__HEADER_H)
#define      __HEADER_H
#define __MODULE_HEADER__

#if !defined(__BASETYPE_H)
#include <rtl_types.h>
#endif


#define MAGIC_FWN       0x4046774eL         /**< @FwN */


#if defined(_C166) || defined(__arm) || defined(F_INFOPAGE_TOOL) || \
    defined(_MCC68K) 
#define GH_TEXT_SIZE    8
#define GH_SPECIAL_SIZE 12
#define MH_TEXT_SIZE    8
#define MH_NAME_SIZE    12
#define MH_SERNO_SIZE   8         /**< including 0 terminator */
#else
#define GH_TEXT_SIZE    8
#define GH_SPECIAL_SIZE 12
#define MH_TEXT_SIZE    8
#define MH_NAME_SIZE    16
#define MH_SERNO_SIZE   8         /**< including 0 terminator */
#endif

#if defined(PRAGMA_PACK)
#pragma pack(1)
#endif

typedef PACKED(struct) tagGlobalHeader
{
    /** here starts the firmware .... */
    uint8_t    startCode[8];                   /**< 0x00 */
    uint32_t   Length;                         /**< FW length */
    uint32_t   magicFWN;                       /**< 0x0C @FwN */
    uint8_t    version[GH_TEXT_SIZE];          /**< 0x10 Vx.yy \0            */
    uint8_t    date[GH_TEXT_SIZE];             /**< 0x18 ddmmyy \0           */
    uint8_t    stollmann[GH_SPECIAL_SIZE];     /**< 0x20 (c) Stollmann GmbH\0*/
    uint8_t    specialString[12];              /**< 0x30 special namestring  */
    uint8_t    endCode[64-GH_SPECIAL_SIZE-GH_TEXT_SIZE-GH_TEXT_SIZE-28];
} TGlobalHeader;
typedef TGlobalHeader  *PGlobalHeader;

typedef PACKED(struct) tagModuleHeader
{
    /** here starts module.... */
    uint8_t    moduleStartCode[8];             /**< 0x40 */
    uint8_t    *firmwareVersion;               /**< 0x48 pointer firmware version */
    uint8_t    reserved[2];                    /**< 0x4C */
    uint8_t    moduleID;                       /**< 0x4E module id           */
    uint8_t    moduleType;                     /**< 0x4f module type         */
    uint8_t    moduleVersion[MH_TEXT_SIZE];    /**< 0x50 Vx.yy \0            */
    uint8_t    moduleDate[MH_TEXT_SIZE];       /**< 0x58 ddmmyy \0           */
    uint8_t    moduleName[MH_NAME_SIZE];       /**< 0x60 name\0              */
    uint8_t    moduleSerialNo[MH_SERNO_SIZE];  /**< 0x6x serial number\0     */
    uint16_t    hardwareType;                   /**< 0x74 TT_...              */
    uint8_t    hardwareSubtype;                /**< 0x75 HW variant          */
    uint8_t    hardwareVersion;                /**< 0x76 HW version          */
    uint16_t    checksum;                       /**< 0x78 for future          */
} TModuleHeader;
typedef TModuleHeader  *PModuleHeader;

typedef PACKED(struct) tagFirmwareHeader
{
    /** here starts the firmware ... */
    TGlobalHeader globalHeader;
    TModuleHeader moduleHeader;
} TFirmwareHeader;
typedef TFirmwareHeader  *PFirmwareHeader;

#if defined(PRAGMA_PACK)
#pragma pack()
#endif

#define GHPTR PGlobalHeader
#define MHPTR PModuleHeader
#define FHPTR PFirmwareHeader

#if defined(_M_I86) || defined(I386)        /**< MSC predefined macro         */
#define NOP1    __asm _emit 0x90
#define NOP2    NOP1 NOP1
#define NOP3    NOP2 NOP1
#define NOP4    NOP2 NOP2

#define MH_MODULE_START \
 __asm jmp ccode NOP3 NOP3 NOP2 ; align to 0x0e
#define MH_ID(x1, x2) \
 __asm _emit x1 __asm _emit x2
#define MH_MAGIC \
 __asm _emit 'N' __asm _emit 'w' __asm _emit 'F' __asm _emit '@'
#define MH_VERSION(x1,x2,x3,x4) \
 __asm _emit 'V' __asm _emit x1 __asm _emit '.' __asm _emit x2 \
 __asm _emit x3 __asm _emit '.' __asm _emit x4 __asm _emit 0
#define MH_DATE(d1,d2,m1,m2,y1,y2) \
 __asm _emit d1 __asm _emit d2 __asm _emit m1 __asm _emit m2 \
 __asm _emit y1 __asm _emit y2 __asm _emit 0  __asm _emit 0
#define MH_TEXT4(x1,x2,x3,x4) \
 __asm _emit x1 __asm _emit x2 __asm _emit x3 __asm _emit x4
#define MH_TEXT8(x1,x2,x3,x4,x5,x6,x7,x8) \
 __asm _emit x1 __asm _emit x2 __asm _emit x3 __asm _emit x4 \
 __asm _emit x5 __asm _emit x6 __asm _emit x7 __asm _emit x8
#define MH_CHECKSUM \
 __asm _emit 0  __asm _emit 0 \
 __asm ccode:

#else

/** other compiler or CPU: */
/** TBD !!!!*/
#define NOP1
#define NOP2    NOP1 NOP1
#define NOP3    NOP2 NOP1
#define NOP4    NOP2 NOP2

#define MH_MODULE_START \

#define MH_ID(x1, x2) \

#define MH_MAGIC \

#define MH_VERSION(x1,x2,x3,x4) \

#define MH_DATE(d1,d2,m1,m2,y1,y2) \

#define MH_TEXT4(x1,x2,x3,x4) \

#define MH_TEXT8(x1,x2,x3,x4,x5,x6,x7,x8) \

#define MH_CHECKSUM \


#endif

extern void  module(void);        /**< for reference to module header */

uint8_t  moduleSearch( FHPTR startAddress, uint16_t kiloBytes, uint8_t  moduleID);


FHPTR getModuleHeader(FHPTR startAddress, uint16_t kiloBytes, uint8_t moduleID);

uint8_t * getFirmwareVersion(void);


FHPTR getBootLoaderHeader( void );
BOOL  getBootLoaderHWInfo(LPWORD hardwareType, uint8_t * hardwareSubtype, uint8_t * hardwareVersion);
uint8_t  bootModuleSearch(FHPTR startAddress, uint16_t kiloBytes, uint16_t userRequest, uint32_t Baudrate);

typedef void ( *TModuleStart)(void);
typedef void ( *TBootStart)(uint16_t RequestType, uint32_t Baudrate);

#endif

